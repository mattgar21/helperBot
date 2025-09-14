# drive_server.py
from flask import Flask, request, jsonify, send_from_directory, abort, Response
import atexit, time, threading

# ------- Motor setup (gpiozero + lgpio) -------
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
PWM_FREQ = 1000

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
    # lc/rc in [-1..+1]
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

# ---------------- Routes ----------------
@app.route("/")
def index():
    # Serve your HTML verbatim
    return send_from_directory(".", "index.html")

@app.route("/status")
def status():
    return jsonify(ok=True, **_snapshot())

# Your page uses /control. Any call here = MANUAL override.
@app.route("/control", methods=["POST"])
def control():
    j = request.get_json(silent=True) or {}
    state = j.get("state")
    direction = j.get("direction")
    speed = int(j.get("speed", 70))

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
        # Return to AUTO so camera can resume immediately
        _set_mode("auto")
        return jsonify(ok=True, **_snapshot())

    abort(400, "Invalid state")

# Camera client posts tank commands here; ignored while in MANUAL
@app.route("/vision_control", methods=["POST"])
def vision_control():
    snap = _snapshot()
    if snap["mode"] != "auto":
        return jsonify(ok=False, reason="manual_override"), 409

    j = request.get_json(silent=True) or {}
    if j.get("state") == "stop":
        stop(); _set_moving("stop"); _touch_vision()
        return jsonify(ok=True, **_snapshot())

    if "left_cmd" in j and "right_cmd" in j:
        tank(j["left_cmd"], j["right_cmd"])
        _set_moving(f"auto tank({j['left_cmd']:.2f},{j['right_cmd']:.2f})")
        _touch_vision()
        return jsonify(ok=True, **_snapshot())

    abort(400, "Missing or invalid vision command")

# ----- NEW: frame ingest + MJPEG stream -----
@app.route("/frame", methods=["POST"])
def receive_frame():
    """
    Vision client posts a single JPEG here (Content-Type: image/jpeg).
    We store only the most recent frame.
    """
    data = request.get_data(cache=False, as_text=False)
    if not data:
        abort(400, "empty")
    set_latest_frame(data)
    return jsonify(ok=True)

@app.route("/video")
def video():
    """
    MJPEG stream of the most recent frames at /video.
    Your HTML can simply use: <img src="/video">
    """
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
            # throttle a bit; effective rate is driven by how fast vision posts
            time.sleep(0.03)

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")
# ---------------------------------------------

if __name__ == "__main__":
    # If you rely on pigpio daemon for timing:
    # sudo systemctl enable --now pigpiod
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
