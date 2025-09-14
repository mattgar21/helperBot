# HelperBot – Autonomous Follower Robot

HelperBot is a Raspberry Pi–powered robot designed to **carry belongings and autonomously follow a designated person** wearing specific attire (e.g., a high-visibility yellow vest).

The system combines **computer vision**, **motor control**, and **ultrasonic sensing** to navigate safely and assist its user in carrying whatever the user chooses.

By **leveraging the TailScale peer-peer mesh network** we are able to connect two linked devices from anywhere in the world to the Raspberry Pi's **Flask webserver.** The HelperBot is not only able to track the user, but it features controllable buttons on the interface that allow the user to control the HelperBot manually.

---

## Features

- 🎥 **Computer Vision Tracking**  
  Detects and follows a person wearing a high-visibility vest using OpenCV.

- 🚗 **Motor Control Server**  
  Web-controlled motor driving via Flask, supporting manual override and autonomous tracking.

- 🌐 **Remote Vision Client**  
  Streams video to a webserver hosted on the Raspberry-Pi. This allows users to access the video stream, toggle tracking, and control the rover manually.

- 🛑 **Safety Systems**  
  Ultrasonic distance sensing for obstacle detection.

- 🧩 **Modular Design**  
  Separate scripts for motor control, vision, testing, and demos.

---

## Repository Structure

    .
    ├── .venv               # Isolates the project’s dependencies.
    │   ├── bin             # Contains virtual environment 
    |   └── lib             # Contains all the libraries used by the program
    ├── index.html          # Contains the HTML and CSS front end design
    ├── motor_control.py    # Starts a Flask server and streams the video
    ├── demo.py             # Test code that's not intended for running on HelperBot
    ├── test.py             # Test code not intended for running on HelperBot in product delivery
    ├── vision_client.py    #
    ├── vision_control_ultrasonic.py   # Runs the vision code in tandem with the ultrasonic sensors
    └── README.md           # Informative file to instruct the user on operating the HelperBot

## How It Works

1. **Vision Client (`vision_client.py`)**
   - Runs on a camera-enabled device (Pi or laptop).
   - Detects yellow vest regions in the video stream.
   - Calculates angle and distance proxy.
   - Sends control commands to the robot’s server.
   - Streams annotated video back for debugging.

2. **Motor Control Server (`motor_control.py`)**
   - Runs on the Raspberry Pi controlling the motors.
   - Exposes REST API (`/control`, `/vision_control`, `/video`).
   - Supports **manual override** via HTTP POST or **automatic follow mode** from vision client.
   - Maintains safety watchdog (stops if no vision updates received).

3. **Test Utility (`test.py`)**
   - Standalone tool for testing yellow vest detection locally.
   - Displays bounding box, confidence, and alignment information in a GUI window.

4. **Demo Script (`demo.py`)**
   - Proof-of-concept showing:
     - Motor GPIO control.
     - Ultrasonic ranging.
     - A simple Flask server for web interface testing.
   - Used to validate hardware setup.

---

## Requirements

- **Hardware**
  - Raspberry Pi (with GPIO motor driver H-bridge and motors).
  - USB/Web camera.
  - Ultrasonic distance sensor (HC-SR04 or similar).
  - Motor driver (L298N, TB6612, etc.).
  - Power supply (batteries).

- **Software**
  - Python 3.x
  - Flask
  - OpenCV
  - NumPy
  - `gpiozero` (with `lgpio`)

### Install dependencies:

    pip install flask opencv-python numpy gpiozero requests


## Usage

### 1. Start Motor Control Server

On the **Raspberry Pi:**

    python motor_control.py

- Server runs on port 5000.

- Visit http://[pi-ip]:5000 for status.

- MJPEG video available at /video.

### 2. Run Vision Client

On Pi or remote laptop with a webcam:

    python vision_client.py

- Detects vest and streams frames to server.
- Sends steering commands automatically.

### 3. Test Vision Locally

For debugging:

    python test.py

- Opens webcam feed with bounding box overlay and telemetry.

### 4. Run Demo (Prototype)

    python demo.py

- Drives motors in pre-programmed patterns.

- Prints ultrasonic distances.

- Serves a small Flask demo page.

---

### Roadmap

- Integrate ultrasonic obstacle avoidance with motor control server.

- Add load-carrying chassis design.

- Improve attire detection (support multiple colors/patterns).

- Expand web interface for remote monitoring and manual control.

### License

This project is for research and educational purposes. License terms TBD.
