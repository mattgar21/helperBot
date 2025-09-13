# track_yellow_vest.py
import cv2
import numpy as np
import time

def main():
    cap = cv2.VideoCapture(1)  # change to 1/2 if you have multiple cameras
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

        cv2.imshow("Yellow Vest Tracker (press q to quit)", out)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
