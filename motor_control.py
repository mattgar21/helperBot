from flask import Flask, render_template, request, jsonify, abort
import RPi.GPIO as GPIO
import atexit

# If index.html is next to app.py, keep template_folder="."
# If you place it in templates/, use template_folder="templates"
app = Flask(__name__, template_folder=".")

# ---------------- Rover Motor Setup ----------------
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 10

EN1 = 9
EN2 = 11



GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up pins
for pin in (IN1, IN2, IN3, IN4, EN1, EN2):
    GPIO.setup(pin, GPIO.OUT)

PWM1 = GPIO.PWM(EN1, 1000)
PWM2 = GPIO.PWM(EN2, 1000)
PWM1.start(0)
PWM2.start(0)

def set_dir(in1, in2, in3, in4):
    GPIO.output(IN1, in1)
    GPIO.output(IN2, in2)
    GPIO.output(IN3, in3)
    GPIO.output(IN4, in4)

def set_speed(lspeed, rspeed):
    ls = max(0, min(100, int(lspeed)))
    rs = max(0, min(100, int(rspeed)))
    PWM1.ChangeDutyCycle(ls)
    PWM2.ChangeDutyCycle(rs)

def backward(speed=70):
    set_dir(0, 1, 1, 0); set_speed(speed, speed)

def forward(speed=70):
    set_dir(1, 0, 0, 1); set_speed(speed, speed)

def left(speed=70):
    set_dir(0, 1, 0, 1); set_speed(speed, speed)

def right(speed=70):
    set_dir(1, 0, 1, 0); set_speed(speed, speed)

def stop():
    set_dir(0, 0, 0, 0); set_speed(0, 0)

def _cleanup():
    try:
        stop()
    finally:
        GPIO.cleanup()

atexit.register(_cleanup)

# ---------------- Routes ----------------
@app.route("/")
def index():
    # Renders index.html from template_folder
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
        return jsonify(ok=True, moving=direction, speed=spd)

    if state == "stop":
        stop()
        return jsonify(ok=True, moving="stop")

    abort(400, "Invalid state")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
