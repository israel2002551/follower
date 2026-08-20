# Follower Robot

Person-following robot control stack with:

- FastAPI web control panel; RTSP video stays on the server.
- Local YOLO person localization.
- MQTT command and telemetry bridge.
- ESP32-CAM sensor/video node.
- ESP32-C3 BTS7960 motor node with command timeout safety.

## Run the Server

```bash
python -m pip install -r requirements.txt
python server.py
```

Open `http://localhost:8000`.

Optional environment variables:

- `ROBOT_ID`, default `sentinel_alpha_99x2`
- `MQTT_BROKER`, default `broker.emqx.io`
- `MQTT_PORT`, default `1883`
- `MQTT_WS_URL`, default `wss://broker.emqx.io:8084/mqtt`, used by the browser task publisher
- `YOLO_MODEL`, default `yolo11n.pt`
- `YOLO_CONFIDENCE`, default `0.45`
- `YOLO_IMAGE_SIZE`, default `320`; lower values run faster, while higher values improve small-person detection
- `TARGET_SMOOTHING`, default `0.55`; higher values react faster, while lower values reduce jitter

On its first run, Ultralytics downloads the configured YOLO weights if they are not already available locally. The server receives RTSP video, runs person detection locally, and never sends camera frames to the dashboard or an external vision API.

## MQTT Task Messages

The dashboard publishes commands to `nodes/<ROBOT_ID>/tasks`. The server accepts JSON messages with these forms:

```json
{"task": "connect_camera", "ip_address": "192.168.1.50"}
{"task": "set_target"}
{"task": "set_mode", "mode": "AUTO"}
{"task": "manual_drive", "action": "FORWARD"}
{"task": "emergency_stop"}
```

The server continues to publish motor commands on `nodes/<ROBOT_ID>/hardware_control` as `{"drive":"FORWARD","speed":150}`. The motor node clamps speed to the safe `0`–`255` PWM range; older messages without `speed` use its default speed.

## Control Logic

The server continuously captures RTSP frames and retains only the newest frame, preventing inference from following an old video backlog. YOLO detects people at a reduced inference size, then keeps a target lock using bounding-box overlap and smooths the target centre before steering. Steering uses separate enter and exit deadbands to avoid left/right oscillation.

The VL53L0X and ultrasonic sensors actively assist the follow decision. The server uses their fused range to maintain following distance, and always uses the nearest valid reading as front-clearance protection. It drives forward only when the target is centred and the front path is clear. When the robot is too close, it stops rather than automatically reversing because the current hardware has no rear obstacle sensor.

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

The motor node also cuts output if no MQTT command arrives within `commandTimeoutMs`.
