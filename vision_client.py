import cv2, numpy as np, math, time, requests

# If running ON the Pi:
SERVER = "http://127.0.0.1:5000"
# If running elsewhere over Tailscale, use the Pi's TS IP or MagicDNS name:
# SERVER = "http://100.x.y.z:5000"  # tailscale ip -4
# SERVER = "http://rover-pi.tail123.ts.net:5000"  # MagicDNS example

# --- Camera FOVs ---
HFOV_DEG = 48.8
VFOV_DEG = 28.6

# --- Detection thresholds (small frame) ---
MIN_VEST_AREA_FRAC = 0.008
MIN_VEST_FILL      = 0.15

# --- Stop distance via area ---
TARGET_AREA_FRAC = 0.050
TARGET_DEADBAND  = 0.010

# --- Controller tuning ---
K_TURN = 0.035
TURN_DEADBAND_DEG = 2.0
K_FWD = 1.6
MIN_FWD_CMD = 0.18
MAX_FWD_CMD = 0.70
MAX_TURN_CMD = 0.65

SEARCH_TURN_CMD = 0.35
SEARCH_BURST_S  = 0.4

# ---------- NEW: JPEG push helper ----------
def post_jpeg(img_bgr, jpeg_quality=70, max_w=640):
    """
    Encode BGR image as JPEG and POST to /frame for the server to stream on /video.
    """
    h, w = img_bgr.shape[:2]
    if w > max_w:
        img_bgr = cv2.resize(img_bgr, (max_w, int(h * (max_w / w))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    if not ok:
        return
    try:
        requests.post(f"{SERVER}/frame", data=buf.tobytes(),
                      headers={"Content-Type": "image/jpeg"}, timeout=0.25)
    except Exception:
        # ignore transient network hiccups
        pass
# -------------------------------------------

def post_vision(left_cmd=None, right_cmd=None, stop_cmd=False):
    url = f"{SERVER}/vision_control"
    if stop_cmd:
        payload = {"state":"stop"}
    else:
        payload = {"left_cmd": float(left_cmd), "right_cmd": float(right_cmd)}
    try:
        r = requests.post(url, json=payload, timeout=0.25)
        if r.status_code == 409:
            # manual override active
            time.sleep(0.15)
            return False
        r.raise_for_status()
        return True
    except Exception:
        time.sleep(0.15)
        return False

def put_text_bottom_left(img, lines, margin=16, scale=0.6, thickness=2, color=(0,255,255)):
    h, w = img.shape[:2]
    y = h - margin
    for line in reversed(lines):
        (tw, th), base = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        cv2.putText(img, line, (margin, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)
        y -= (th + base + 4)

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")

    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    except Exception:
        pass

    alpha = 0.2
    conf_smooth = 0.0
    scale = 0.75
    last_seen = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        out = frame.copy()  # draw overlays on this
        H_img, W_img = frame.shape[:2]

        # center crosshair
        img_cx, img_cy = W_img // 2, H_img // 2
        cv2.drawMarker(out, (img_cx, img_cy), (255,255,255), cv2.MARKER_CROSS, 24, 2)

        # process at smaller scale
        small = cv2.resize(frame, (int(W_img*scale), int(H_img*scale)))
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([22, 90, 120], np.uint8)
        upper_yellow = np.array([40, 255, 255], np.uint8)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, 1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, 2)

        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        vest_box, vest_area = None, 0
        for c in cnts:
            a = cv2.contourArea(c)
            if a > vest_area: vest_area = a; vest_box = cv2.boundingRect(c)

        now = time.time()
        hud = []

        if vest_box is None:
            conf_smooth = (1 - alpha) * conf_smooth
            hud = [f"Vest: {int(conf_smooth*100)}%", "Searching..."]
            # spin search occasionally
            if (now - last_seen) > 0.4:
                post_vision(left_cmd=+SEARCH_TURN_CMD, right_cmd=-SEARCH_TURN_CMD)
                # draw hint arrow
                cv2.arrowedLine(out, (img_cx-60, img_cy), (img_cx-100, img_cy), (0,255,255), 3, tipLength=0.4)
                time.sleep(SEARCH_BURST_S)
            # send current annotated frame
            put_text_bottom_left(out, hud)
            post_jpeg(out)
            # optional local preview:
            # cv2.imshow("Tracker", out); cv2.waitKey(1)
            continue

        x,y,ww,hh = vest_box
        roi_mask = mask[y:y+hh, x:x+ww]
        filled = float(cv2.countNonZero(roi_mask))
        box_area = max(ww*hh, 1)
        conf = np.clip(filled/box_area, 0.0, 1.0)

        small_area = float(mask.shape[0] * mask.shape[1])
        area_frac  = (ww * hh) / small_area
        passes_mag = (area_frac >= MIN_VEST_AREA_FRAC) and (conf >= MIN_VEST_FILL)

        if not passes_mag:
            conf_smooth = (1 - alpha) * conf_smooth
            hud = [f"Vest: {int(conf_smooth*100)}%", "Weak candidate (size/fill low)"]
            put_text_bottom_left(out, hud)
            post_jpeg(out)
            # cv2.imshow("Tracker", out); cv2.waitKey(1)
            continue

        # accepted detection
        conf_smooth = (1 - alpha) * conf_smooth + alpha * conf
        last_seen = now

        inv = 1.0 / scale
        X = int(x * inv); Y = int(y * inv)
        W = int(ww * inv); H = int(hh * inv)

        cx = X + W/2.0
        cy = Y + H/2.0

        # draw bbox + center line
        cv2.rectangle(out, (X, Y), (X+W, Y+H), (0,255,255), 3)
        cv2.line(out, (img_cx, img_cy), (int(cx), int(cy)), (0,255,255), 2)
        cv2.circle(out, (int(cx), int(cy)), 5, (0,255,255), -1)

        dx_px = cx - img_cx
        nx = dx_px / (W_img/2.0)
        ang_x_deg = nx * (HFOV_DEG / 2.0)

        # distance proxy via area
        area_err = (TARGET_AREA_FRAC - area_frac)

        # turning
        if abs(ang_x_deg) <= TURN_DEADBAND_DEG:
            turn_cmd = 0.0
        else:
            turn_cmd = float(np.clip(K_TURN * ang_x_deg, -MAX_TURN_CMD, MAX_TURN_CMD))

        # forward
        if area_frac >= (TARGET_AREA_FRAC - TARGET_DEADBAND):
            fwd_cmd = 0.0
        else:
            fwd_cmd = float(np.clip(K_FWD * area_err, MIN_FWD_CMD, MAX_FWD_CMD))

        left_cmd  = fwd_cmd - turn_cmd
        right_cmd = fwd_cmd + turn_cmd
        post_vision(left_cmd=left_cmd, right_cmd=right_cmd)

        hud = [
            f"Vest: {int(conf_smooth*100)}%",
            f"angle X: {ang_x_deg:+.2f}°",
            f"area: {area_frac:.3%} (target {TARGET_AREA_FRAC:.1%})",
            f"cmd L,R: {left_cmd:+.2f}, {right_cmd:+.2f}"
        ]
        put_text_bottom_left(out, hud)

        # push the annotated frame for /video
        post_jpeg(out)

        # Optional local preview
        # cv2.imshow("Tracker", out)
        # if (cv2.waitKey(1) & 0xFF) == ord('q'):
        #     break

        time.sleep(0.02)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
