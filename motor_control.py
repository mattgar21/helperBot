from flask import Flask, render_template, request, jsonify, abort 
import atexit

from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, Motor
Device.pin_factory = LGPIOFactory()


# If index.html is next to app.py, keep template_folder="."
# If you place it in templates/, use template_folder="templates"
app = Flask(__name__, template_folder=".")

# ---------------- Rover Motor Setup (BCM) ----------------
IN1 = 17   # Left motor: forward
IN2 = 27   # Left motor: backward
IN3 = 22   # Right motor: backward
IN4 = 10   # Right motor: forward

EN1 = 9    # Left enable (PWM)
EN2 = 11   # Right enable (PWM)

PWM_FREQ = 1000  # Hz

# Use pigpio daemon as backend (make sure pigpiod is running)

# Create Motor objects; pwm=True puts a PWMLED on the enable pin
left_motor = Motor(forward=IN1, backward=IN2, enable=EN1, pwm=True)
right_motor = Motor(forward=IN4, backward=IN3, enable=EN2, pwm=True)

# Optionally bump frequency (supported by pigpio backend)
try:
    # Access underlying PWMLED via private attribute (safe for pigpio backend)
    left_motor._speed._device._frequency = PWM_FREQ
    right_motor._speed._device._frequency = PWM_FREQ
except Exception:
    # If backend doesn’t expose frequency like this, it will still work
    pass

def _clamp_pct(p):
    return max(0, min(100, int(p)))

def _pct_to_unit(p):
    # gpiozero Motor speed takes 0.0–1.0
    return _clamp_pct(p) / 100.0

def backward(speed=70):
    s = _pct_to_unit(speed)
    left_motor.backward(s)
    right_motor.backward(s)

def forward(speed=70):
    s = _pct_to_unit(speed)
    left_motor.forward(s)
    right_motor.forward(s)

def left(speed=70):
    s = _pct_to_unit(speed)
    left_motor.backward(s)
    right_motor.forward(s)

def right(speed=70):
    s = _pct_to_unit(speed)
    left_motor.forward(s)
    right_motor.backward(s)

def stop():
    left_motor.stop()
    right_motor.stop()

def _cleanup():
    try:
        stop()
    finally:
        # Free GPIO gracefully
        left_motor.close()
        right_motor.close()

atexit.register(_cleanup)

# ---------------- Routes ----------------
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/control", methods=["POST"])
def control():
    print("is_json:", request.is_json)
    print("raw data:", request.data)
    print("json:", request.get_json(silent=True))

    data = request.get_json(silent=True) or {}
    state = data.get("state")
    direction = data.get("direction")
    spd = int(data.get("speed", 70))

    if state == "start":
        if direction == "forward":
            forward(spd)
        elif direction == "backward":
            backward(spd)
        elif direction == "left":
            left(spd)
        elif direction == "right":
            right(spd)
        else:
            abort(400, "Unknown direction")
        return jsonify(ok=True, moving=direction, speed=_clamp_pct(spd))

    if state == "stop":
        stop()
        return jsonify(ok=True, moving="stop")

    abort(400, "Invalid state")

if __name__ == "__main__":
    # No reloader to avoid double-initialization
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
