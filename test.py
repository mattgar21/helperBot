import cv2
import numpy as np
import math
import time

# --- Camera FOVs (set to your webcam's specs if you know them) ---
HFOV_DEG = 48.8
VFOV_DEG = 28.6

# --- Detection thresholds (on the *small* frame) ---
MIN_VEST_AREA_FRAC = 0.008   # 0.8% of the small frame area
MIN_VEST_FILL      = 0.15    # at least 15% of the bbox must be yellow

def put_text_bottom_left(img, lines, margin=16, scale=0.7, thickness=2, color=(0,255,255)):
    """
    Draw multiple text lines stacked upward from the bottom-left corner.
    """
    h, w = img.shape[:2]
    y = h - margin
    for line in reversed(lines):
        (tw, th), baseline = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        y_line = max(th + baseline, 0)
        cv2.putText(img, line, (margin, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)
        y -= (th + baseline + 6)

def main():
    cap = cv2.VideoCapture(0)  # <-- change index if needed (0,1,2...)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam. Try a different index (0/1/2) or check permissions.")

    # Optional exposure tweaks (may not be supported by your driver)
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    except Exception:
        pass

    conf_smooth = 0.0
    alpha = 0.2
    scale = 0.75  # downscale for processing speed
    print("Entering main loop")

    center_aligned_x = False
    center_aligned_y = False
    center_inside_vest = False

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        H_img, W_img = frame.shape[:2]
        small = cv2.resize(frame, (int(W_img*scale), int(H_img*scale)), interpolation=cv2.INTER_LINEAR)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        # --- Yellow high-vis mask ---
        lower_yellow = np.array([22, 90, 120], dtype=np.uint8)
        upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        vest_box = None
        vest_area = 0

        for c in cnts:
            area = cv2.contourArea(c)
            if area > vest_area:
                vest_area = area
                vest_box = cv2.boundingRect(c)

        out = frame.copy()

        # Draw a white center crosshair
        img_cx, img_cy = W_img/2.0, H_img/2.0
        cv2.drawMarker(out, (int(img_cx), int(img_cy)), (255,255,255),
                       markerType=cv2.MARKER_CROSS, markerSize=22, thickness=2)

        info_lines = []

        if vest_box is not None:
            x, y, ww, hh = vest_box
            inv = 1.0 / scale
            X = int(x * inv); Y = int(y * inv)
            W = int(ww * inv); H = int(hh * inv)

            # Confidence (fill ratio of mask in the box, computed on the small mask)
            roi_mask = mask[y:y+hh, x:x+ww]
            filled = float(cv2.countNonZero(roi_mask))
            box_area = max(ww * hh, 1)
            conf = np.clip(filled / box_area, 0.0, 1.0)

            # ---- Magnitude gating: require enough yellow to count as a "vest" ----
            small_area = float(mask.shape[0] * mask.shape[1])
            area_frac  = (ww * hh) / small_area
            passes_mag = (area_frac >= MIN_VEST_AREA_FRAC) and (conf >= MIN_VEST_FILL)

            if not passes_mag:
                # Treat as "no vest" this frame
                conf_smooth = (1 - alpha) * conf_smooth
                info_lines = [
                    f"Vest: {int(conf_smooth*100)}%",
                    "dx, dy (px):   0,   0  | dist: 0px",
                    "dx, dy (norm): +0.000, +0.000",
                    "angle X,Y: +0.00°, +0.00°",
                    "center X in box: False",
                    "center inside box: False",
                    f"(filtered: area {area_frac:.3%} < {MIN_VEST_AREA_FRAC:.3%} "
                    f"or fill {conf:.2f} < {MIN_VEST_FILL:.2f})"
                ]
            else:
                # Update smoothed confidence only when we actually accept the detection
                conf_smooth = (1 - alpha) * conf_smooth + alpha * conf

                # Draw box
                cv2.rectangle(out, (X, Y), (X+W, Y+H), (0,255,255), 3)

                # Center of box (original scale)
                cx = X + W/2.0
                cy = Y + H/2.0

                # Vector from image center to target
                dx_px = cx - img_cx
                dy_px = cy - img_cy
                dist_px = math.hypot(dx_px, dy_px)

                # Normalized offsets (–1..1 at image edges)
                nx = dx_px / (W_img/2.0)
                ny = dy_px / (H_img/2.0)

                # Angular offsets from FOV
                ang_x_deg = nx * (HFOV_DEG / 2.0)
                ang_y_deg = ny * (VFOV_DEG / 2.0)

                # Draw center-to-target line and target dot
                cv2.line(out, (int(img_cx), int(img_cy)), (int(cx), int(cy)), (0,255,255), 2)
                cv2.circle(out, (int(cx), int(cy)), 5, (0,255,255), -1)

                # --- Flags: is the image center inside the box? ---
                center_aligned_x = (X <= img_cx <= X + W)
                center_aligned_y = (Y <= img_cy <= Y + H)
                center_inside_vest = center_aligned_x and center_aligned_y

                # Bottom-left HUD
                info_lines = [
                    f"Vest: {int(conf_smooth*100)}%",
                    f"dx, dy (px): {dx_px:+.0f}, {dy_px:+.0f}  | dist: {dist_px:.0f}px",
                    f"dx, dy (norm): {nx:+.3f}, {ny:+.3f}",
                    f"angle X,Y: {ang_x_deg:+.2f}\u00b0, {ang_y_deg:+.2f}\u00b0",
                    f"center X in box: {center_aligned_x}",
                    f"center inside box: {center_inside_vest}",
                    f"[area {area_frac:.3%} | fill {conf:.2f}]"
                ]
        else:
            conf_smooth = (1 - alpha) * conf_smooth
            # Reset flags when no vest candidate at all
            center_aligned_x = False
            center_aligned_y = False
            center_inside_vest = False

            info_lines = [
                f"Vest: {int(conf_smooth*100)}%",
                "dx, dy (px):   0,   0  | dist: 0px",
                "dx, dy (norm): +0.000, +0.000",
                "angle X,Y: +0.00°, +0.00°",
                "center X in box: False",
                "center inside box: False"
            ]

        # Small mask preview (top-left)
        mask_vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_vis = cv2.resize(mask_vis, (out.shape[1]//4, out.shape[0]//4))
        out[0:mask_vis.shape[0], 0:mask_vis.shape[1]] = mask_vis

        # Draw the info stack bottom-left
        put_text_bottom_left(out, info_lines, margin=16, scale=0.7, thickness=2, color=(0,255,255))

        cv2.imshow("Yellow Vest Tracker (q to quit)", out)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break
    print("Closing")
    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
