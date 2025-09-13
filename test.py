# track_yellow_vest.py
import cv2
import numpy as np
import time

def main():
    print("Started")
    cap = cv2.VideoCapture(1)  # replace 1 with the working index
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam. Try a different index (1, 2) or check permissions.")

    # Optional: try to reduce exposure flicker (not supported on all webcams)
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 ~ manual on some drivers
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)        # adjust as needed
    except Exception:
        pass

    # Use an exponential moving average to smooth confidence readout
    conf_smooth = 0.0
    alpha = 0.2
    print("Entering main loop")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed; retrying...")
            time.sleep(0.01)
            continue

        # Work on a smaller copy for speed, keep a scale factor for drawing
        h, w = frame.shape[:2]
        scale = 0.75
        small = cv2.resize(frame, (int(w*scale), int(h*scale)), interpolation=cv2.INTER_LINEAR)

        # Convert to HSV (better for color thresholding)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        # ---- Color mask for high-vis YELLOW ----
        # Typical neon yellow (greenish) vest sits around H=25..40.
        # You might need to tweak these if lighting is unusual.
        # S and V are kept relatively high to avoid dull yellows.
        lower_yellow = np.array([22, 90, 120], dtype=np.uint8)
        upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Find the largest blob (assume it's the vest)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        vest_box = None
        vest_area = 0

        for c in cnts:
            area = cv2.contourArea(c)
            if area > vest_area:
                vest_area = area
                vest_box = cv2.boundingRect(c)  # (x, y, w, h)

        # Draw results on the original-sized frame
        out = frame.copy()
        if vest_box is not None:
            x, y, ww, hh = vest_box
            # Rescale box to original frame size
            inv = 1.0 / scale
            X = int(x * inv); Y = int(y * inv)
            W = int(ww * inv); H = int(hh * inv)

            # Compute a simple confidence: how filled the box is with the target color
            # (fraction of mask pixels inside the box)
            roi_mask = mask[y:y+hh, x:x+ww]
            filled = float(cv2.countNonZero(roi_mask))
            box_area = max(ww * hh, 1)
            conf = np.clip(filled / box_area, 0.0, 1.0)
            conf_smooth = (1 - alpha) * conf_smooth + alpha * conf

            print("Distance from center")
            """
            Determining the distance from the center of the camera
            """
            import math

            # Assume you already have: out (original frame), and X,Y,W,H for the box
            H_img, W_img = out.shape[:2]

            # 1) Centers
            cx = X + W/2.0
            cy = Y + H/2.0
            img_cx = W_img/2.0
            img_cy = H_img/2.0

            # 2) Pixel offsets (positive dx = right of center, positive dy = below center)
            dx_px = cx - img_cx
            dy_px = cy - img_cy
            dist_px = math.hypot(dx_px, dy_px)

            # 3) Normalized offsets (–1..1, independent of resolution)
            nx = dx_px / (W_img/2.0)
            ny = dy_px / (H_img/2.0)

            # 4a) Angular offsets using FIELD OF VIEW (easy method)
            # Set your camera FOVs (check spec sheet; common values: hfov≈70°, vfov≈43°)
            HFOV_DEG = 70.0
            VFOV_DEG = 43.0
            ang_x_deg = nx * (HFOV_DEG / 2.0)
            ang_y_deg = ny * (VFOV_DEG / 2.0)

            # 4b) (Optional, more accurate) Using intrinsics fx, fy if you calibrated the camera:
            # fx, fy = ...  # focal length in pixels from calibration
            # ang_x_deg = math.degrees(math.atan2(dx_px, fx))
            # ang_y_deg = math.degrees(math.atan2(dy_px, fy))

            # Draw a crosshair at image center and at box center
            cv2.drawMarker(out, (int(img_cx), int(img_cy)), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            cv2.circle(out, (int(cx), int(cy)), 5, (0, 255, 255), -1)

            # Visual line from center to target
            cv2.line(out, (int(img_cx), int(img_cy)), (int(cx), int(cy)), (0, 255, 255), 2)

            # Overlay readout
            info = [
                f"dx, dy (px): {dx_px:+.0f}, {dy_px:+.0f}  | dist: {dist_px:.0f}px",
                f"dx, dy (norm): {nx:+.3f}, {ny:+.3f}",
                f"angle X,Y: {ang_x_deg:+.2f}°, {ang_y_deg:+.2f}°"
            ]
            y0 = 30
            for i, t in enumerate(info):
                cv2.putText(out, t, (20, y0 + i*28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


            """
            End
            """
            print("Drawing box")
            # Draw
            cv2.rectangle(out, (X, Y), (X + W, Y + H), (0, 255, 255), 3)
            label = f"Vest: {int(conf_smooth*100)}%"
            cv2.putText(out, label, (X, max(30, Y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        else:
            conf_smooth = (1 - alpha) * conf_smooth  # decay toward 0
            cv2.putText(out, "Vest: 0%", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

        # Small diagnostics pane (mask preview)
        mask_vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_vis = cv2.resize(mask_vis, (out.shape[1]//4, out.shape[0]//4))
        out[0:mask_vis.shape[0], 0:mask_vis.shape[1]] = mask_vis

        print("Showing camera pov")
        cv2.imshow("Yellow Vest Tracker (press q to quit)", out)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
