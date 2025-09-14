# drive_server.py
from flask import Flask, request, jsonify, send_from_directory, abort, Response
import atexit, time, threading

# ------- Motor setup (gpiozero + your PWM source) -------
# If you want steadier PWM, consider pigpio:
# from gpiozero.pins.pigpio import PiGPIOFactory
# Device.pin_factory = PiGPIOFactory()  # requires pigpiod
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, Motor
Device.pin_factory = LGPIOFactory()

# BCM pins
IN1 = 17   # Left forward
IN2 = 27   # Left backward
IN3 = 22   # Right backward
IN4 = 10   # Right forward
EN1 = 9    # Left enable (PWM)
EN2 = 11   # Right enable (PWM)

PWM_FREQ = 60  # Hz per your hardware

left_motor  = Motor(forward=IN1, backward=IN2, enable=EN1, pwm=True)
right_motor = Motor(forward=IN4, backward=IN3, enable=EN2, pwm=True)
try:
    left_motor._speed._device._frequency  = PWM_FREQ
    right_motor._speed._device._frequency = PWM_FREQ
except Exception:
    pass

def _clamp01(x): return max(0.0, min(1.0, float(x)))

def stop():
    left_motor.stop(); right_motor.stop()

def forward(pct):
    s = _clamp01(int(pct)/100.0)
    left_motor.forward(s); right_motor.forward(s)

def backward(pct):
    s = _clamp01(int(pct)/100.0)
    left_motor.backward(s); right_motor.backward(s)

def left(pct):
    s = _clamp01(int(pct)/100.0)
    left_motor.backward(s); right_motor.forward(s)

def right(pct):
    s = _clamp01(int(pct)/100.0)
    left_motor.forward(s); right_motor.backward(s)

def tank(lc, rc):
    """
    lc/rc in [-1..+1] duty. This clamps and applies sign to both motors.
    """
    lc = max(-1.0, min(1.0, float(lc)))
    rc = max(-1.0, min(1.0, float(rc)))
    if lc >= 0: left_motor.forward(_clamp01(lc))
    else:       left_motor.backward(_clamp01(-lc))
    if rc >= 0: right_motor.forward(_clamp01(rc))
    else:       right_motor.backward(_clamp01(-rc))

def _cleanup():
    try: stop()
    finally:
        left_motor.close(); right_motor.close()
atexit.register(_cleanup)

# --------------- App & State ----------------
app = Flask(__name__, template_folder=".")
state_lock = threading.Lock()

MODE = "auto"          # "auto" or "manual"
MOVING = "stop"
LAST_VISION_TS = 0.0
WATCHDOG_TIMEOUT = 1.0  # seconds; only applies in AUTO

def _set_mode(m):
    global MODE
    with state_lock:
        MODE = m

def _touch_vision():
    global LAST_VISION_TS
    with state_lock:
        LAST_VISION_TS = time.time()

def _set_moving(txt):
    global MOVING
    with state_lock:
        MOVING = txt

def _snapshot():
    with state_lock:
        return {
            "mode": MODE,
            "moving": MOVING,
            "since_vision": time.time() - LAST_VISION_TS
        }

# Watchdog: if in AUTO and camera goes quiet, stop.
def _watchdog():
    while True:
        time.sleep(0.1)
        snap = _snapshot()
        if snap["mode"] == "auto" and snap["since_vision"] > WATCHDOG_TIMEOUT:
            stop(); _set_moving("stop (auto watchdog)")
threading.Thread(target=_watchdog, daemon=True).start()

# ---- camera frame cache for MJPEG ----
_latest_frame = None
_frame_lock = threading.Lock()

def set_latest_frame(jpeg_bytes: bytes):
    global _latest_frame
    with _frame_lock:
        _latest_frame = jpeg_bytes

def get_latest_frame():
    with _frame_lock:
        return _latest_frame
# --------------------------------------

# --------- SERVER-SIDE FOLLOW TUNING ---------
# Turning (common)
K_TURN_SERVER      = 0.035   # deg -> turn command
TURN_DEADBAND_DEG  = 2.0
STEER_SIGN         = +1.0    # set to -1.0 if steering is inverted
MAX_TURN_CMD       = 0.65

# Forward from Y position (preferred if provided by client)
# 0.0 = top, 1.0 = bottom. We move forward at FULL DUTY when below this band.
TARGET_Y_NORM      = 0.35
Y_DEADBAND         = 0.05

# Legacy forward from area (fallback, full duty when too small)
TARGET_AREA_FRAC   = 0.050
TARGET_DEADBAND    = 0.010
# ---------------------------------------------------

# ---------------- Routes ----------------
@app.route("/")
def index():
    # Serve your HTML
    return send_from_directory(".", "index.html")

@app.route("/status")
def status():
    return jsonify(ok=True, **_snapshot())

# Any call here = MANUAL override.
@app.route("/control", methods=["POST"])
def control():
    j = request.get_json(silent=True) or {}
    state = j.get("state")
    direction = j.get("direction")
    speed = int(j.get("speed", 100))  # UI passes 30..100; manual honors it

    _set_mode("manual")

    if state == "start":
        if direction == "forward":    forward(speed);  _set_moving(f"manual forward@{speed}")
        elif direction == "backward": backward(speed); _set_moving(f"manual backward@{speed}")
        elif direction == "left":     left(speed);     _set_moving(f"manual left@{speed}")
        elif direction == "right":    right(speed);    _set_moving(f"manual right@{speed}")
        else: abort(400, "Unknown direction")
        return jsonify(ok=True, **_snapshot())

    if state == "stop":
        stop(); _set_moving("stop")
        _set_mode("auto")  # return to auto so vision can resume
        return jsonify(ok=True, **_snapshot())

    abort(400, "Invalid state")

# Camera client posts commands or measurements here; ignored in MANUAL
@app.route("/vision_control", methods=["POST"])
def vision_control():
    snap = _snapshot()
    if snap["mode"] != "auto":
        return jsonify(ok=False, reason="manual_override"), 409

    j = request.get_json(silent=True) or {}

    # --- PRIORITY 1: raw tank commands (unchanged/back-compat) ---
    if "left_cmd" in j and "right_cmd" in j:
        lc = float(j["left_cmd"]); rc = float(j["right_cmd"])
        tank(lc, rc)
        _set_moving(f"auto tank({lc:.2f},{rc:.2f})")
        _touch_vision()
        return jsonify(ok=True, **_snapshot())

    # --- PRIORITY 2: measurements for server-side chase ---
    # Inputs we may receive:
    # ang_x_deg: horizontal angle error (deg, +right/-left)
    # y_norm:    vertical center normalized [0..1] (0=top, 1=bottom)
    # or cy + H_img (or H) to compute y_norm = cy / H_img
    # area_frac: optional legacy distance proxy (fallback)

    ang_x_deg = float(j.get("ang_x_deg", 0.0))

    # TURN: proportional with deadband, sign-corrected
    if abs(ang_x_deg) <= TURN_DEADBAND_DEG:
        turn_cmd = 0.0
    else:
        turn_cmd = max(-MAX_TURN_CMD, min(MAX_TURN_CMD, STEER_SIGN * K_TURN_SERVER * ang_x_deg))

    # Determine y_norm if provided (preferred)
    y_norm = None
    if "y_norm" in j:
        try:
            y_norm = float(j["y_norm"])
        except Exception:
            y_norm = None
    elif "cy" in j:
        cy = float(j["cy"])
        H_img = float(j.get("H_img", j.get("H", 0.0)))
        if H_img > 0:
            y_norm = max(0.0, min(1.0, cy / H_img))

    # ---- NEW: forward = "manual max" when moving forward ----
    # We emulate your web-app at 100%: if we decide to move forward, we use full duty (1.0).
    fwd_full = 1.0
    fwd_cmd  = 0.0
    method   = "turnOnly"

    if y_norm is not None:
        # If the box center is below the target band (middle/bottom), go FULL forward.
        if y_norm > (TARGET_Y_NORM - Y_DEADBAND):
            fwd_cmd = fwd_full
        else:
            fwd_cmd = 0.0
        method = f"followY(y_norm={y_norm:.2f})"

    elif "area_frac" in j:
        # Legacy area-based: if target looks too small, go FULL forward.
        area_frac = float(j.get("area_frac", 0.0))
        if area_frac < (TARGET_AREA_FRAC - TARGET_DEADBAND):
            fwd_cmd = fwd_full
        else:
            fwd_cmd = 0.0
        method = f"followArea(area={area_frac:.3f})"

    # Mix: forward at full duty when needed, plus turning authority
    lc = fwd_cmd - turn_cmd
    rc = fwd_cmd + turn_cmd
    # Clamp just in case
    lc = max(-1.0, min(1.0, lc))
    rc = max(-1.0, min(1.0, rc))

    tank(lc, rc)
    _set_moving(f"auto {method} l={lc:.2f}, r={rc:.2f}, ang={ang_x_deg:.2f}, fwd={'1.0' if fwd_cmd>0 else '0.0'}")
    _touch_vision()
    return jsonify(ok=True, **_snapshot())

# ----- frame ingest + MJPEG stream -----
@app.route("/frame", methods=["POST"])
def receive_frame():
    data = request.get_data(cache=False, as_text=False)
    if not data:
        abort(400, "empty")
    set_latest_frame(data)
    return jsonify(ok=True)

@app.route("/video")
def video():
    boundary = b"--frame"
    def gen():
        while True:
            jf = get_latest_frame()
            if jf is None:
                time.sleep(0.05)
                continue
            yield boundary + b"\r\n"
            yield b"Content-Type: image/jpeg\r\n"
            yield f"Content-Length: {len(jf)}\r\n\r\n".encode("ascii")
            yield jf + b"\r\n"
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")
# ---------------------------------------------

if __name__ == "__main__":
    # If you switch to pigpio timing for steadier PWM:
    # sudo systemctl enable --now pigpiod
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
