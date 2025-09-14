import cv2, numpy as np, math, time, requests, threading

# ========= Pin factory for gpiozero (LGPIO) =========
# Requires: python-lgpio (and lgpio kernel module available)
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, DistanceSensor
Device.pin_factory = LGPIOFactory()
# ====================================================

# ====================== CONFIG ======================
SERVER = "http://127.0.0.1:5000"

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

MAX_TURN_CMD = 0.65
MIN_FWD_CMD   = 0.35
MAX_FWD_CMD   = 1.30

SEARCH_TURN_CMD = 0.35
SEARCH_BURST_S  = 0.4

# --- Ultrasonic config (gpiozero DistanceSensor) ---
ECHO_PIN = 5
TRIG_PIN = 6
ULTRA_POLL_S    = 0.10          # 10 Hz
STOP_THRESH_CM  = 30.0
TRIP_COUNT      = 4             # >=4 consecutive to trip/clear
MAX_DISTANCE_M  = 2.0           # sensor cap (meters) for scaling/filtering
# ====================================================

# ---------- JPEG push helper ----------
def post_jpeg(img_bgr, jpeg_quality=70, max_w=640):
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
        pass

def post_vision(left_cmd=None, right_cmd=None, stop_cmd=False):
    url = f"{SERVER}/vision_control"
    if stop_cmd:
        payload = {"state":"stop"}
    else:
        payload = {"left_cmd": float(left_cmd), "right_cmd": float(right_cmd)}
    try:
        r = requests.post(url, json=payload, timeout=0.25)
        if r.status_code == 409:  # manual override active
            time.sleep(0.15); return False
        r.raise_for_status()
        return True
    except Exception:
        time.sleep(0.15); return False

def put_text_bottom_left(img, lines, margin=16, scale=0.6, thickness=2, color=(0,255,255)):
    h, w = img.shape[:2]
    y = h - margin
    for line in reversed(lines):
        (tw, th), base = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        cv2.putText(img, line, (margin, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)
        y -= (th + base + 4)

# ===================== Ultrasonic (gpiozero) =====================
class UltrasonicGuard:
    """
    gpiozero.DistanceSensor reader in its own thread.
    Trip STOP when <= 30 cm occurs >= TRIP_COUNT times consecutively.
    Release when > 30 cm occurs >= TRIP_COUNT times consecutively.
    """
    def __init__(self,
                 echo_pin=ECHO_PIN, trig_pin=TRIG_PIN,
                 poll_s=ULTRA_POLL_S,
                 stop_thresh_cm=STOP_THRESH_CM, trip_count=TRIP_COUNT,
                 max_distance_m=MAX_DISTANCE_M):
        self.poll_s = poll_s
        self.stop_thresh_cm = stop_thresh_cm
        self.trip_count = trip_count

        self.stopped_event = threading.Event()
        self._below_ctr = 0
        self._above_ctr = 0
        self._last_cm = None
        self._thread = None
        self._stop_req = threading.Event()
        self._sensor = None

        try:
            # DistanceSensor.distance returns meters (0..max_distance)
            self._sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin,
                                          max_distance=max_distance_m,
                                          queue_len=3)  # light smoothing
        except Exception as e:
            print(f"[Ultrasonic] Failed to init gpiozero DistanceSensor: {e}")
            self._sensor = None

    def _loop(self):
        next_heartbeat = 0
        while not self._stop_req.is_set():
            if self._sensor is None:
                time.sleep(self.poll_s)
                continue
            try:
                d_m = float(self._sensor.distance)  # meters, 0..max_distance
                # clamp weird reads
                if d_m < 0: d_m = 0.0
                if d_m > MAX_DISTANCE_M: d_m = MAX_DISTANCE_M
                d_cm = d_m * 100.0
                self._last_cm = d_cm

                if d_cm <= self.stop_thresh_cm:
                    self._below_ctr += 1
                    self._above_ctr = 0
                else:
                    self._above_ctr += 1
                    self._below_ctr = 0

                # Trip STOP latch
                if (not self.stopped_event.is_set()) and (self._below_ctr >= self.trip_count):
                    self.stopped_event.set()
                    print(f"[Ultrasonic] STOPPED (<= {self.stop_thresh_cm:.0f} cm x{self._below_ctr}).")
                    post_vision(stop_cmd=True)  # immediate stop

                # Release latch
                if self.stopped_event.is_set() and (self._above_ctr >= self.trip_count):
                    self.stopped_event.clear()
                    print(f"[Ultrasonic] CLEARED (> {self.stop_thresh_cm:.0f} cm x{self._above_ctr}).")

            except Exception as e:
                # Ignore a single failure and keep going
                print(f"[Ultrasonic] Read error: {e}")

            # While stopped, send a soft heartbeat stop
            if self.stopped_event.is_set():
                now = time.time()
                if now >= next_heartbeat:
                    post_vision(stop_cmd=True)
                    next_heartbeat = now + 0.5

            time.sleep(self.poll_s)

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop_req.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            if self._sensor is not None:
                self._sensor.close()
        except Exception:
            pass

    @property
    def stopped(self):
        return self.stopped_event.is_set()

    @property
    def last_distance_cm(self):
        return self._last_cm

# =================== CV main program ===================
def main():
    ultra = UltrasonicGuard()
    ultra.start()

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
    last_stop_beat = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue

            out = frame.copy()
            H_img, W_img = frame.shape[:2]

            # crosshair
            img_cx, img_cy = W_img // 2, H_img // 2
            cv2.drawMarker(out, (img_cx, img_cy), (255,255,255), cv2.MARKER_CROSS, 24, 2)

            # small processing frame
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

            # ----- Ultrasonic STOP gate -----
            stopped = ultra.stopped
            dlast = ultra.last_distance_cm

            if stopped:
                if (now - last_stop_beat) > 0.5:
                    post_vision(stop_cmd=True)
                    last_stop_beat = now

            if vest_box is None:
                conf_smooth = (1 - alpha) * conf_smooth
                status = "STOPPED (ultrasonic)" if stopped else "Searching..."
                hud = [f"Vest: {int(conf_smooth*100)}%", status]
                if dlast is not None:
                    hud.append(f"US: {dlast:.1f} cm")

                if (not stopped) and ((now - last_seen) > 0.4):
                    post_vision(left_cmd=+SEARCH_TURN_CMD, right_cmd=-SEARCH_TURN_CMD)
                    cv2.arrowedLine(out, (img_cx-60, img_cy), (img_cx-100, img_cy), (0,255,255), 3, tipLength=0.4)
                    time.sleep(SEARCH_BURST_S)

                put_text_bottom_left(out, hud)
                post_jpeg(out)
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
                status = "STOPPED (ultrasonic)" if stopped else "Weak candidate (size/fill low)"
                hud = [f"Vest: {int(conf_smooth*100)}%", status]
                if dlast is not None:
                    hud.append(f"US: {dlast:.1f} cm")
                put_text_bottom_left(out, hud)
                post_jpeg(out)
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

            # ---------- TURNING ----------
            if abs(ang_x_deg) <= TURN_DEADBAND_DEG:
                turn_cmd = 0.0
            else:
                turn_cmd = float(np.clip(K_TURN * ang_x_deg, -MAX_TURN_CMD, MAX_TURN_CMD))

            # ---------- FORWARD: Y-based speed ----------
            if area_frac >= (TARGET_AREA_FRAC - TARGET_DEADBAND):
                fwd_cmd = 0.0
            else:
                y_pos = float(np.clip(cy / H_img, 0.0, 1.0))
                GAMMA_Y = 1.2
                fwd_cmd = MIN_FWD_CMD + (MAX_FWD_CMD - MIN_FWD_CMD) * (y_pos ** GAMMA_Y)
                if abs(ang_x_deg) < 5.0:
                    fwd_cmd = min(fwd_cmd + 0.10, MAX_FWD_CMD)
                BOTTOM_STOP_FRAC = 0.90
                if y_pos >= BOTTOM_STOP_FRAC:
                    fwd_cmd = 0.0

            left_cmd  = fwd_cmd + turn_cmd
            right_cmd = fwd_cmd - turn_cmd

            if stopped:
                post_vision(stop_cmd=True)
                status = "STOPPED (ultrasonic)"
                hud = [
                    f"Vest: {int(conf_smooth*100)}%",
                    status,
                    f"US: {dlast:.1f} cm" if dlast is not None else "US: n/a",
                    f"angle X: {ang_x_deg:+.2f}°",
                    f"area: {area_frac:.3%} (target {TARGET_AREA_FRAC:.1%})",
                    f"y_pos: {cy/H_img:.2f}",
                    f"cmd L,R: {left_cmd:+.2f}, {right_cmd:+.2f} (suppressed)"
                ]
                put_text_bottom_left(out, hud)
                post_jpeg(out)
                time.sleep(0.02)
                continue

            # normal motion
            post_vision(left_cmd=left_cmd, right_cmd=right_cmd)

            hud = [
                f"Vest: {int(conf_smooth*100)}%",
                f"US: {dlast:.1f} cm" if dlast is not None else "US: n/a",
                f"angle X: {ang_x_deg:+.2f}°",
                f"area: {area_frac:.3%} (target {TARGET_AREA_FRAC:.1%})",
                f"y_pos: {cy/H_img:.2f}",
                f"cmd L,R: {left_cmd:+.2f}, {right_cmd:+.2f}"
            ]
            put_text_bottom_left(out, hud)
            post_jpeg(out)
            time.sleep(0.02)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        try: ultra.stop()
        except Exception: pass

if __name__ == "__main__":
    main()
