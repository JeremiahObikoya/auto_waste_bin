# Auto Waste Bin

An autonomous mobile waste-bin prototype built with ESP32 microcontrollers,
real-time sensors, wireless communication, and AI-assisted computer vision.
The system can be driven manually, operate through an autonomous patrol mode,
or use the ESP32-CAM and a laptop vision server to approach a person carrying
waste and verify obstacles during navigation.

## Project Overview

The project is split into three cooperating parts:

1. **Main ESP32 navigation controller**
	 - Controls the four DC motors through an L298N motor driver.
	 - Reads distance data from an ultrasonic sensor.
	 - Uses an MPU6050 IMU for heading and orientation feedback.
	 - Controls the ultrasonic sensor servo and camera pan servo.
	 - Hosts the robot web dashboard and exposes telemetry and control endpoints.
	 - Communicates with the ESP32-CAM over Wi-Fi UDP.

2. **ESP32-CAM vision controller**
	 - Captures QVGA camera frames.
	 - Sends frames to the laptop vision server over HTTP.
	 - Receives vision decisions and sends state updates to the main ESP32.
	 - Coordinates scanning, tracking, interaction, release, and obstacle-check states.

3. **Laptop Python vision server**
	 - Receives JPEG frames at `/process_frame`.
	 - Sends image and state-specific prompts to Gemini.
	 - Returns structured detection results to the ESP32-CAM.
	 - Provides a live dashboard with the latest frame, response, latency, and errors.

## Operating Modes

The main ESP32 dashboard provides three operating modes:

- **Manual mode**: Drive the robot with the web dashboard while obstacle safety
	checks remain active.
- **Autonomous mode**: Run the classroom patrol state machine using navigation,
	heading, distance, and motor-control logic.
- **CAM Guided mode**: Use the ESP32-CAM and Gemini vision results to scan for a
	person holding waste, track the target, verify proximity, and distinguish the
	target from an unrelated obstacle.

## Vision Backends

Two laptop server implementations are included:

### Gemini REST API

`src/main/server.py` sends camera frames directly to the Gemini REST API. It is
the direct API integration and uses the model and server settings in
`src/main/config.py`.

### Gemini Web Chat Automation

`src/main/server_web_scraper.py` connects Selenium to a Chrome instance running
with remote debugging. It uploads each frame to an already signed-in Gemini web
session and extracts the response. This mode is started with
`src/main/launch_chrome.bat`.

Both servers use the same HTTP contract and can return detections such as:

- `EXTENDED_ARM_WITH_OBJECT`
- `TRACKING`
- `STOP_PALM`
- `PROXIMITY`
- `EMPTY_ARM`
- `BACK_TURNED`
- `IS_TARGET`
- `IS_OBSTACLE`
- `NONE`

## Hardware

### Main controller

- ESP32 Dev Module, 
- L298N motor driver
- Four DC motors
- HC-SR04-style ultrasonic sensor
- Servo for ultrasonic sensor scanning
- Servo for ESP32-CAM pan control
- MPU6050 IMU

### Vision controller

- AI Thinker ESP32-CAM
- OV2640 camera module

The primary pin assignments are documented at the top of
`src/main/esp_main/esp_main.ino`.

## Repository Layout

```text
.
├── README.md
├── .gitignore
└── src
		├── main
		│   ├── esp_main
		│   │   ├── esp_main.ino
		│   │   └── secrets.example.h
		│   ├── esp_cam_main
		│   │   ├── esp_cam_main.ino
		│   │   └── secrets.example.h
		│   ├── server.py
		│   ├── server_web_scraper.py
		│   ├── config.py
		│   ├── requirements.txt
		│   └── launch_chrome.bat
		└── tests
				├── integrated_test
				├── gemini_api_control
				├── manual_control_test
				├── mpu6050_calibration
				└── test_*
```

The `tests` directory contains standalone Arduino sketches for testing the
ESP32 boards, camera, motors, servos, ultrasonic sensor, MPU6050, integration,
and earlier Gemini control experiments.

## Python Setup

From the repository root, create and activate a virtual environment, then
install the laptop server dependencies:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r src\main\requirements.txt
```

Run the direct API server with:

```powershell
Set-Location src\main
python server.py
```

The server listens on port `5000` and exposes the dashboard at
`http://localhost:5000`.

## Gemini Web Chat Setup

To use the browser automation backend:

1. Run `src/main/launch_chrome.bat`.
2. Sign in to Gemini in the Chrome window that opens.
3. Keep that Chrome window running.
4. Start the server from `src/main`:

```powershell
Set-Location src\main
python server_web_scraper.py
```

Chrome remote debugging uses port `9222`. The ESP32-CAM must be able to reach
the laptop on the same network at port `5000`.

## Arduino Setup

1. Install the ESP32 board package in the Arduino IDE.
2. Install the libraries used by the main controller:
	 - Adafruit MPU6050
	 - Adafruit Unified Sensor
	 - ESP32Servo
3. Open each maintained sketch from its own folder.
4. Copy `secrets.example.h` to `secrets.h` in both maintained firmware folders.
5. Fill in the local Wi-Fi, access-point, laptop IP, and communication settings.
6. Select the appropriate board and port, then compile and upload.

For the ESP32-CAM, use the AI Thinker ESP32-CAM board configuration. The
maintained camera sketch specifies the required camera pin mapping and flash
settings in its header comments.

## Network Communication

- The main ESP32 hosts its control dashboard over Wi-Fi.
- The ESP32-CAM posts JPEG frames to:
	`http://<laptop-ip>:5000/process_frame`.
- The laptop server returns a JSON vision decision.
- The two ESP32 boards exchange control and vision state through UDP.
- The default UDP ports are `8888` and `8889`; these are configured in the
	firmware secrets templates.

## Main Server Endpoints

The laptop vision servers provide:

- `POST /process_frame`: Receive a JPEG frame and return a vision decision.
- `GET /latest_frame.jpg`: Return the most recent camera frame.
- `GET /video_feed`: Stream the latest camera frame.
- `GET /api/status`: Return dashboard state, detections, latency, and errors.
- `GET /`: Open the live monitoring dashboard.

The main ESP32 provides its own dashboard and control endpoints, including
`/api/telemetry`, `/api/cmd`, and `/api/settings`.

## Development and Testing

The Arduino sketches under `src/tests` are intended for hardware validation
before running the complete system. Recommended order:

1. Test the ESP32 board and camera.
2. Calibrate and test the MPU6050.
3. Test the motors, servos, and ultrasonic sensor independently.
4. Run the integrated controller test.
5. Flash the maintained main and camera firmware.
6. Start one laptop vision backend and test CAM Guided mode.

Python syntax can be checked from the repository root with:

```powershell
python -m compileall -q src\main
```

## Current Scope

This repository contains the firmware, laptop vision servers, hardware test
sketches, and configuration templates for the prototype. The system is intended
for controlled local-network testing with the robot and its connected hardware.
