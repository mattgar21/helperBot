import threading
import time
import RPi.GPIO as GPIO
from flask import Flask

app = Flask(__name__)

def web():
    try:
        while True:
            @app.route("/")
            def home():
                return "Hello, Flask is working!"

            @app.route("/test")
            def test():
                return "This is a test route."

            if __name__ == "__main__":
                app.run(debug=True)
    except RuntimeError:
        pass

IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 10

EN1 = 9
EN2 = 11

ECHO = 5
TRIG = 6

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
MOTOR_PINS = [IN1, IN2, IN3, IN4, EN1, EN2]
GPIO.setup(MOTOR_PINS, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(TRIG, GPIO.OUT)

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
    PWM1.ChangeDutyCycle(lspeed)
    PWM2.ChangeDutyCycle(rspeed)

def forward(speed=100):
    set_dir(0, 1, 1, 0)
    set_speed(speed, speed)

def backward(speed=100):
    set_dir(1, 0, 0, 1)
    set_speed(speed, speed)

def turn_left(speed=100):
    set_dir(0, 1, 0, 1)
    set_speed(speed, speed)

def turn_right(speed=100):
    set_dir(1, 0, 1, 0)
    set_speed(speed, speed)

def stop():
    set_speed(0, 0)
    set_dir(0, 0, 0, 0)

def drive():
    try:
        while True:
            print("[Drive] Forward")
            forward(70)
            time.sleep(2)

            print("[Drive] Turn Right")
            turn_right(60)
            time.sleep(1)

            print("[Drive] Backward")
            backward(50)
            time.sleep(2)

            print("[Drive] Stop")
            stop()
            time.sleep(2)

    except KeyboardInterrupt:
        stop()
        GPIO.cleanup()
        print("Drive thread stopped.")


# CV Tracking Here
def track():
    while True:
        time.sleep(0.5)

# Ultrasonic Thread
def ultrasonic(timeout=0.02):
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()

    while GPIO.input(ECHO) == 0:
        start = time.time()
        if start - start_time > timeout:
            raise RuntimeError("Echo Start Timeout")

    pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        end = time.time()
        if end - pulse_start > timeout:
            raise RuntimeError("Echo End Timeout")
        
    pulse_end = time.time()
    
    duration = pulse_end - pulse_start
    distance = (duration * 34300) / 2 #cm
    return distance

def ultrasonic_thread(stop_event):
    while not stop_event.is_set():
        try:
            distance = ultrasonic()
            print(f"Ultrasonic Sensor: {distance:.2f} cm")
            time.sleep(0.1)
        except RuntimeError:
            pass


if __name__ =="__main__":
    drive_thread = threading.Thread(target=drive, daemon=True)
    tracking_thread = threading.Thread(target=track, daemon=True)    
    stop_event = threading.Event()
    distance_thread = threading.Thread(target=ultrasonic_thread, args=(stop_event,))
    web_thread = threading.Thread(target=web, daemon=True)


    distance_thread.start()
    drive_thread.start()
    tracking_thread.start()
    web_thread.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutting down")
        stop_event.set()
        distance_thread.join()
        stop()
        GPIO.cleanup()

    print("Done!")
