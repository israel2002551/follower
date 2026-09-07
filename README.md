# Follower Robot

Person-following robot control stack with:

- Static MQTT web control panel; RTSP video stays on the server.
- Local YOLO person localization.
- MQTT command and telemetry bridge.
- ESP32-CAM sensor/video node.
- ESP32-C3 BTS7960 motor node with command timeout safety.

## Run the Server

```bash
python -m pip install -r requirements.txt
python server.py
```

Host `index.html` as a static page (for example, GitHub Pages). The dashboard sends tasks to the server only through MQTT and receives dashboard state from MQTT.

Optional environment variables:

- `ROBOT_ID`, default `sentinel_alpha_99x2`
- `MQTT_BROKER`, default `broker.emqx.io`
- `MQTT_PORT`, default `1883`
- `SHOW_STREAM`, default `true`; set to `0` or `false` to disable the video display window on headless systems
- `FOLLOW_MIN_MM`, default `350` (35 cm safety stop threshold)
- `FOLLOW_TARGET_MM`, default `1000` (1.0 m preferred follow distance)
- `FOLLOW_MAX_MM`, default `2200` (2.2 m max follow speed ramp limit)
- `AUTO_MIN_SPEED`, default `150` (minimum forward speed to overcome motor friction)
- `AUTO_MAX_SPEED`, default `215` (maximum autonomous tracking speed)
- `AUTO_TURN_SPEED`, default `170` (skid-steer turning speed)
- `MANUAL_DRIVE_SPEED`, default `180` (manual WASD drive speed)
- `YOLO_MODEL`, default `yolo11n.pt`
- `YOLO_CONFIDENCE`, default `0.45`
- `YOLO_IMAGE_SIZE`, default `320`; lower values run faster, while higher values improve small-person detection
- `TARGET_SMOOTHING`, default `0.55`; higher values react faster, while lower values reduce jitter
- `AUTO_FULL_SPEED`, default `255`; full PWM speed for sensor-based follow mode when YOLO has no box
- `SENSOR_FOLLOW_MAX_MM`, default `2500`; maximum range for sensor-only person following

### Live Vision GUI Window

When running `python server.py`, a live OpenCV window opens showing:
- Real-time RTSP camera stream.
- High-tech YOLO bounding box with confidence score and distance tags for the tracked person.
- Candidate bounding boxes in amber for any other detected individuals in frame.
- Center guideline, steering deadband thresholds (`L`, `CENTER`, `R`), and target bearing vector.
- Comprehensive telemetry HUD: Mode (`AUTO`/`MANUAL`), Drive Command & PWM Speed, Tracking Source (`YOLO`/`SENSOR`/`NONE`), Laser / Sonic / Fused distance telemetry, and Vision FPS / Inference latency.
- Keyboard Shortcuts:
  - `W` / `A` / `S` / `D`: Drive Forward / Left / Backward / Right (in `MANUAL` mode).
  - `M`: Toggle between `AUTO` (following) and `MANUAL` modes.
  - `SPACE` or `X`: Stop motors.
  - `Q` or `ESC`: Graceful shutdown (halts motors, closes window, disconnects MQTT).

Set the matching `mqttWebSocketUrl` and `robotId` constants in `index.html` when you use a different broker or robot ID.

On its first run, Ultralytics downloads the configured YOLO weights if they are not already available locally. The server receives RTSP video, runs person detection locally, and never sends camera frames to the dashboard or an external vision API.

## MQTT Task Messages

The dashboard publishes camera and AUTO-mode tasks to `nodes/<ROBOT_ID>/tasks`, and the server publishes retained state snapshots to `nodes/<ROBOT_ID>/dashboard_status`. The web dashboard subscribes to that status topic, so it does not need an HTTP endpoint. In Manual mode, the dashboard publishes motor commands directly to `nodes/<ROBOT_ID>/hardware_control`; the ESP32-C3 motor node subscribes to that topic. The server accepts JSON task messages with these forms:

```json
{"task": "connect_camera", "ip_address": "192.168.1.50"}
{"task": "set_target"}
{"task": "set_mode", "mode": "AUTO"}
{"task": "emergency_stop"}
```

In Manual mode, the browser publishes `{"drive":"FORWARD","speed":170}` straight to the motor topic. In AUTO mode, the server publishes motor commands on that same topic. The motor node clamps speed to the safe `0`–`255` PWM range; older messages without `speed` use its default speed.

## Control Logic

The server continuously captures RTSP frames and retains only the newest frame, preventing inference from following an old video backlog. YOLO detects people at a reduced inference size, then keeps a target lock using bounding-box overlap and smooths the target centre before steering. Steering uses separate enter and exit deadbands to avoid left/right oscillation.

### Dual Tracking & Sensor Fallback
- **YOLO Detected**: When the YOLO bounding box detects the person, steering aligns the robot with the person and forward speed smoothly ramps between `AUTO_MIN_SPEED` and `AUTO_MAX_SPEED` based on distance.
- **YOLO Absent (Sensor Tracking)**: If the YOLO box does not detect the person, the server immediately engages sensor tracking using the fused VL53L0X Laser and Ultrasonic telemetry. If the person is ahead (`distance > FOLLOW_TARGET_MM`), the cart drives **FORWARD at full speed** (`AUTO_FULL_SPEED = 255`).
- **Collision Safety**: Regardless of tracking mode, if the front clearance drops below `FOLLOW_MIN_MM` (320 mm), the motors immediately cut to `STOP` (0 PWM). When the robot is within preferred distance, it holds position without reversing.

Forward speed ramps from `AUTO_MIN_SPEED` near the preferred follow distance to `AUTO_MAX_SPEED` at the far-distance limit. Turns use the lower `AUTO_TURN_SPEED`, which reduces overshoot. Manual commands use `MANUAL_DRIVE_SPEED` unless the caller includes a specific speed.

## Hardware Setup

1. Flash `main.c` to the ESP32-CAM sensor/video node.
2. Flash `motor_node.ino` to the ESP32-C3 motor node.
3. Set the WiFi credentials in `main.c` and `motor_node.ino` so both nodes join the same network.
4. The ESP32-C3 subscribes directly to `nodes/sentinel_alpha_99x2/hardware_control` over MQTT; ESP-NOW is not used.
5. Connect the web UI to the ESP32-CAM IP address.
6. Use a voltage divider or level shifter between the ultrasonic sensor's `ECHO` pin and the ESP32-CAM `SONIC_ECHO` pin; ESP32 GPIOs accept only 3.3 V.

## Safety Tuning

The server follows only a detected person in AUTO mode and stops when YOLO confidence is low, the target is stale, or distance telemetry is invalid. Tune these constants in `server.py` for your robot speed and sensor placement:

- `TURN_ENTER_DEADBAND_PX` and `TURN_EXIT_DEADBAND_PX`
- `FOLLOW_MIN_MM`
- `FOLLOW_TARGET_MM`
- `FOLLOW_MAX_MM`
- `TARGET_STALE_SEC`
- `AUTO_FULL_SPEED`
- `SENSOR_FOLLOW_MAX_MM`

The motor node also cuts output if no MQTT command arrives within `commandTimeoutMs`.
