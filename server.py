import asyncio
import json
import os
import time

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from ultralytics import YOLO


ROBOT_ID = os.getenv("ROBOT_ID", "sentinel_alpha_99x2")
MQTT_BROKER = os.getenv("MQTT_BROKER", "broker.emqx.io")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
CAMERA_IP = os.getenv("CAMERA_IP", "10.26.101.104")
CAMERA_URL = os.getenv("CAMERA_URL", "rtsp://10.26.101.104:554/")
CONTROL_TOPIC = f"nodes/{ROBOT_ID}/hardware_control"
TELEMETRY_TOPIC = f"nodes/{ROBOT_ID}/telemetry"
TASK_TOPIC = f"nodes/{ROBOT_ID}/tasks"
STATUS_TOPIC = f"nodes/{ROBOT_ID}/dashboard_status"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2
TURN_ENTER_DEADBAND_PX = 70
TURN_EXIT_DEADBAND_PX = 40
FOLLOW_MIN_MM = int(os.getenv("FOLLOW_MIN_MM", "300"))
FOLLOW_TARGET_MM = int(os.getenv("FOLLOW_TARGET_MM", "650"))
FOLLOW_MAX_MM = int(os.getenv("FOLLOW_MAX_MM", "1800"))
AUTO_MIN_SPEED = int(os.getenv("AUTO_MIN_SPEED", "155"))
AUTO_MAX_SPEED = int(os.getenv("AUTO_MAX_SPEED", "220"))
AUTO_FULL_SPEED = int(os.getenv("AUTO_FULL_SPEED", "255"))
AUTO_TURN_SPEED = int(os.getenv("AUTO_TURN_SPEED", "175"))
MANUAL_DRIVE_SPEED = int(os.getenv("MANUAL_DRIVE_SPEED", "180"))
SENSOR_FOLLOW_MAX_MM = int(os.getenv("SENSOR_FOLLOW_MAX_MM", "3000"))
TELEMETRY_STALE_SEC = 2.0
TARGET_STALE_SEC = 1.8
AI_INTERVAL_SEC = 0.08
COMMAND_MIN_INTERVAL_SEC = 0.18
YOLO_MODEL_PATH = os.getenv("YOLO_MODEL", "yolo11n.pt")
YOLO_CONFIDENCE = float(os.getenv("YOLO_CONFIDENCE", "0.45"))
YOLO_IMAGE_SIZE = int(os.getenv("YOLO_IMAGE_SIZE", "320"))
TARGET_SMOOTHING = float(os.getenv("TARGET_SMOOTHING", "0.55"))
SHOW_STREAM = os.getenv("SHOW_STREAM", "true").lower() in ("1", "true", "yes", "y")
WINDOW_TITLE = f"Follower Robot - YOLO Vision ({ROBOT_ID})"

rtsp_cap = None
yolo_model = None
mqtt_loop_started = False
server_loop = None
stop_event = None
latest_frame = None
latest_frame_id = 0
tracking_box = None
smoothed_center = None
steering_direction = None
state_lock = asyncio.Lock()
state = {
    "mode": "AUTO",
    "connected": False,
    "target_profile": {"class": "person"},
    "last_target": None,
    "last_target_seen_at": 0.0,
    "last_sensor_target_seen_at": 0.0,
    "tracking_source": "NONE",
    "last_ai_started_at": 0.0,
    "last_command": "STOP",
    "last_speed": 0,
    "last_command_at": 0.0,
    "last_reason": "Waiting for camera connection.",
    "mqtt_connected": False,
    "last_task_at": 0.0,
    "vision_fps": 0.0,
    "inference_ms": None,
    "telemetry": {"sonic_mm": None, "front_clearance_mm": None, "received_at": 0.0},
}


def now():
    return time.monotonic()


def valid_distance(distance_mm, low, high):
    try:
        return low < float(distance_mm) < high
    except (TypeError, ValueError):
        return False


def telemetry_age():
    received_at = state["telemetry"].get("received_at") or 0.0
    return now() - received_at if received_at else None


def process_ultrasonic_distance(sonic_mm):
    """Validate and return cleaned ultrasonic distance reading in mm."""
    if valid_distance(sonic_mm, 30, 4000):
        return round(float(sonic_mm), 1)
    return None


def get_yolo_model():
    global yolo_model
    if yolo_model is None:
        yolo_model = YOLO(YOLO_MODEL_PATH)
    return yolo_model


def box_iou(first_box, second_box):
    left = max(first_box[0], second_box[0])
    top = max(first_box[1], second_box[1])
    right = min(first_box[2], second_box[2])
    bottom = min(first_box[3], second_box[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    if not intersection:
        return 0.0
    first_area = (first_box[2] - first_box[0]) * (first_box[3] - first_box[1])
    second_area = (second_box[2] - second_box[0]) * (second_box[3] - second_box[1])
    return intersection / max(first_area + second_area - intersection, 1)


def select_target(candidates):
    global tracking_box, smoothed_center
    if tracking_box is None:
        selected = max(candidates, key=lambda candidate: candidate["confidence"] * candidate["area"])
    else:
        selected = max(
            candidates,
            key=lambda candidate: (2.0 * box_iou(candidate["box"], tracking_box)) + candidate["confidence"],
        )
    tracking_box = selected["box"]
    if smoothed_center is None:
        smoothed_center = (selected["x"], selected["y"])
    else:
        old_x, old_y = smoothed_center
        smoothed_center = (
            (TARGET_SMOOTHING * selected["x"]) + ((1 - TARGET_SMOOTHING) * old_x),
            (TARGET_SMOOTHING * selected["y"]) + ((1 - TARGET_SMOOTHING) * old_y),
        )
    return {
        "visible": True,
        "x": round(smoothed_center[0], 1),
        "y": round(smoothed_center[1], 1),
        "confidence": selected["confidence"],
        "box": [round(value, 1) for value in selected["box"]],
    }


def reset_tracking():
    global tracking_box, smoothed_center, steering_direction
    tracking_box = None
    smoothed_center = None
    steering_direction = None


def read_person_target(frame):
    model = get_yolo_model()
    result = model.predict(frame, classes=[0], conf=YOLO_CONFIDENCE, imgsz=YOLO_IMAGE_SIZE, verbose=False)[0]
    if result.boxes is None or len(result.boxes) == 0:
        return {"visible": False, "x": None, "y": None, "confidence": 0.0, "box": None, "all_candidates": []}

    candidates = []
    for box, confidence in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist()):
        x1, y1, x2, y2 = [float(value) for value in box]
        candidates.append({
            "box": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
            "x": round((x1 + x2) / 2, 1),
            "y": round((y1 + y2) / 2, 1),
            "area": round(max(1.0, (x2 - x1) * (y2 - y1)), 1),
            "confidence": round(float(confidence), 3),
        })
    selected = select_target(candidates)
    selected["all_candidates"] = candidates
    return selected


def create_placeholder_frame():
    """Generates an informative dark standby screen when camera stream is opening."""
    placeholder = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    placeholder[:] = (20, 24, 28)

    title = f"FOLLOWER ROBOT ({ROBOT_ID})"
    cv2.putText(placeholder, title, (FRAME_CENTER_X - 180, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 220, 180), 2, cv2.LINE_AA)

    status_text = "Connecting to RTSP camera stream..."
    cv2.putText(placeholder, status_text, (FRAME_CENTER_X - 160, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)

    cam_url = CAMERA_URL if CAMERA_URL else CAMERA_IP
    url_text = f"URL: {cam_url}"
    cv2.putText(placeholder, url_text, (FRAME_CENTER_X - 140, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 170, 180), 1, cv2.LINE_AA)

    cv2.putText(placeholder, "[Q] Quit  |  [M] Toggle Mode", (FRAME_CENTER_X - 120, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 130, 140), 1, cv2.LINE_AA)
    return placeholder


def draw_hud(frame, target, telemetry, state_snapshot):
    """Draws YOLO bounding boxes, steering deadbands, telemetry, and status HUD onto the video frame."""
    # 1. Guidance and deadband lines
    cv2.line(frame, (FRAME_CENTER_X, 55), (FRAME_CENTER_X, FRAME_HEIGHT - 35), (70, 70, 70), 1, cv2.LINE_AA)
    left_deadband = FRAME_CENTER_X - TURN_ENTER_DEADBAND_PX
    right_deadband = FRAME_CENTER_X + TURN_ENTER_DEADBAND_PX
    cv2.line(frame, (left_deadband, 55), (left_deadband, FRAME_HEIGHT - 35), (90, 80, 50), 1, cv2.LINE_AA)
    cv2.line(frame, (right_deadband, 55), (right_deadband, FRAME_HEIGHT - 35), (90, 80, 50), 1, cv2.LINE_AA)
    cv2.putText(frame, "L", (left_deadband - 14, FRAME_HEIGHT - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 140, 140), 1, cv2.LINE_AA)
    cv2.putText(frame, "CENTER", (FRAME_CENTER_X - 22, FRAME_HEIGHT - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 140, 140), 1, cv2.LINE_AA)
    cv2.putText(frame, "R", (right_deadband + 6, FRAME_HEIGHT - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 140, 140), 1, cv2.LINE_AA)

    # 2. YOLO Bounding Boxes
    if target:
        all_candidates = target.get("all_candidates", [])
        is_target_fresh = bool(target.get("visible")) and (now() - (state_snapshot.get("last_target_seen_at") or 0.0) < 0.8)
        tracked_box = target.get("box") if is_target_fresh else None

        # Draw candidate boxes in subtle orange
        for cand in all_candidates:
            c_box = cand.get("box")
            if not c_box:
                continue
            is_tracked = False
            if tracked_box and abs(c_box[0] - tracked_box[0]) < 8 and abs(c_box[1] - tracked_box[1]) < 8:
                is_tracked = True
            if not is_tracked:
                cx1, cy1, cx2, cy2 = [int(v) for v in c_box]
                cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), (0, 165, 255), 1)
                lbl = f"Person {int(cand.get('confidence', 0) * 100)}%"
                cv2.putText(frame, lbl, (cx1, max(14, cy1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1, cv2.LINE_AA)

        # Draw tracked target with high-visibility brackets, center crosshair, and telemetry tag
        if is_target_fresh and tracked_box:
            x1, y1, x2, y2 = [int(v) for v in tracked_box]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(FRAME_WIDTH - 1, x2), min(FRAME_HEIGHT - 1, y2)

            box_color = (50, 255, 50)
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
            corner_len = min(20, (x2 - x1) // 4, (y2 - y1) // 4)
            if corner_len > 4:
                cv2.line(frame, (x1, y1), (x1 + corner_len, y1), box_color, 3)
                cv2.line(frame, (x1, y1), (x1, y1 + corner_len), box_color, 3)
                cv2.line(frame, (x2, y1), (x2 - corner_len, y1), box_color, 3)
                cv2.line(frame, (x2, y1), (x2, y1 + corner_len), box_color, 3)
                cv2.line(frame, (x1, y2), (x1 + corner_len, y2), box_color, 3)
                cv2.line(frame, (x1, y2), (x1, y2 - corner_len), box_color, 3)
                cv2.line(frame, (x2, y2), (x2 - corner_len, y2), box_color, 3)
                cv2.line(frame, (x2, y2), (x2 - corner_len, y2), box_color, 3)

            tx, ty = int(target["x"]), int(target["y"])
            cv2.drawMarker(frame, (tx, ty), box_color, cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)
            cv2.circle(frame, (tx, ty), 5, box_color, 1, cv2.LINE_AA)
            cv2.line(frame, (FRAME_CENTER_X, FRAME_HEIGHT - 35), (tx, ty), (0, 220, 100), 1, cv2.LINE_AA)

            conf_pct = int(target.get("confidence", 0) * 100)
            sonic_dist = telemetry.get("sonic_mm")
            badge_text = f"TARGET {conf_pct}%"
            if sonic_dist is not None:
                badge_text += f" | {int(sonic_dist)}mm"
            (tw, th), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            badge_y2 = max(th + 6, y1 - 4)
            badge_y1 = badge_y2 - th - 6
            badge_x2 = min(FRAME_WIDTH - 2, x1 + tw + 10)
            cv2.rectangle(frame, (x1, badge_y1), (badge_x2, badge_y2), box_color, -1)
            cv2.putText(frame, badge_text, (x1 + 5, badge_y2 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)

    # 3. Translucent HUD Banners (Top & Bottom)
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (FRAME_WIDTH, 50), (16, 20, 25), -1)
    cv2.rectangle(overlay, (0, FRAME_HEIGHT - 32), (FRAME_WIDTH, FRAME_HEIGHT), (16, 20, 25), -1)
    cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)

    # Top Bar: Mode, Command, Tracking, MQTT
    mode = state_snapshot.get("mode", "AUTO")
    mode_color = (200, 200, 50) if mode == "AUTO" else (60, 160, 255)
    cv2.putText(frame, f"MODE: {mode}", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, mode_color, 2, cv2.LINE_AA)

    cmd = state_snapshot.get("last_command", "STOP")
    spd = state_snapshot.get("last_speed", 0)
    cmd_color = (50, 255, 50) if cmd == "FORWARD" else ((0, 220, 255) if cmd in ("LEFT", "RIGHT") else (80, 80, 240))
    cv2.putText(frame, f"CMD: {cmd} ({spd})", (160, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, cmd_color, 2, cv2.LINE_AA)

    src = state_snapshot.get("tracking_source", "NONE")
    src_color = (50, 255, 50) if src == "YOLO" else ((0, 220, 255) if src == "SENSOR" else (120, 120, 120))
    cv2.putText(frame, f"TRACK: {src}", (350, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, src_color, 2, cv2.LINE_AA)

    mqtt_ok = state_snapshot.get("mqtt_connected", False)
    cv2.putText(frame, "MQTT: ON" if mqtt_ok else "MQTT: OFF", (520, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (50, 255, 50) if mqtt_ok else (60, 60, 240), 1, cv2.LINE_AA)

    sonic = f"{int(telemetry['sonic_mm'])}mm" if telemetry.get("sonic_mm") is not None else "--"
    cv2.putText(frame, f"Ultrasonic: {sonic}", (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 220, 240), 1, cv2.LINE_AA)

    # Bottom Bar: FPS, Inference latency, Reason, Hotkeys
    fps = state_snapshot.get("vision_fps", 0.0)
    inf_ms = state_snapshot.get("inference_ms")
    inf_str = f"{inf_ms}ms" if inf_ms is not None else "--"
    cv2.putText(frame, f"FPS: {fps:.1f} ({inf_str})", (12, FRAME_HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 220, 180), 1, cv2.LINE_AA)

    reason = str(state_snapshot.get("last_reason", ""))
    if len(reason) > 36:
        reason = reason[:33] + "..."
    cv2.putText(frame, reason, (165, FRAME_HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 190, 200), 1, cv2.LINE_AA)

    hint_text = "[W/A/S/D] Drive [M] Auto [Q] Quit" if mode == "MANUAL" else "[M] Manual Mode [Q] Quit"
    cv2.putText(frame, hint_text, (FRAME_WIDTH - 215, FRAME_HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 160, 170), 1, cv2.LINE_AA)


def on_message(client, userdata, msg):
    if msg.topic == TELEMETRY_TOPIC:
        try:
            payload = json.loads(msg.payload.decode())
            sonic = payload.get("sonic_mm")
            dist = process_ultrasonic_distance(sonic)
            state["telemetry"] = {
                "sonic_mm": dist,
                "front_clearance_mm": dist,
                "received_at": now(),
            }
        except Exception as exc:
            print(f"Telemetry parsing error: {exc}")
    elif msg.topic == TASK_TOPIC and server_loop is not None:
        try:
            task = json.loads(msg.payload.decode())
            asyncio.run_coroutine_threadsafe(handle_task(task), server_loop)
        except Exception as exc:
            print(f"Task parsing error: {exc}")


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"[MQTT] Connected successfully to {MQTT_BROKER}:{MQTT_PORT}")
        client.subscribe([(TELEMETRY_TOPIC, 0), (TASK_TOPIC, 1)])
        state["mqtt_connected"] = True
    else:
        print(f"[MQTT] Connection failed with code {rc}")
        state["mqtt_connected"] = False


def on_disconnect(client, userdata, *args):
    print("[MQTT] Disconnected from broker. Auto-reconnecting in background...")
    state["mqtt_connected"] = False


try:
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"FollowerServer_{ROBOT_ID}")
except AttributeError:
    mqtt_client = mqtt.Client(client_id=f"FollowerServer_{ROBOT_ID}")

mqtt_client.reconnect_delay_set(min_delay=1, max_delay=5)
mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect
mqtt_client.on_message = on_message


def connect_mqtt():
    global mqtt_loop_started
    try:
        mqtt_client.connect_async(MQTT_BROKER, MQTT_PORT, keepalive=30)
        if not mqtt_loop_started:
            mqtt_client.loop_start()
            mqtt_loop_started = True
        return True
    except Exception as exc:
        state["mqtt_connected"] = False
        state["last_reason"] = f"MQTT broker unavailable: {exc}"
        print(f"[MQTT] Warning: {state['last_reason']}")
        return False


def clamp_speed(speed):
    try:
        return max(0, min(255, int(speed)))
    except (TypeError, ValueError):
        return 0


def forward_speed(distance_mm):
    distance_error = max(0.0, min(float(distance_mm) - FOLLOW_TARGET_MM, FOLLOW_MAX_MM - FOLLOW_TARGET_MM))
    ratio = distance_error / max(FOLLOW_MAX_MM - FOLLOW_TARGET_MM, 1)
    return round(AUTO_MIN_SPEED + (ratio * (AUTO_MAX_SPEED - AUTO_MIN_SPEED)))


async def publish_drive(action, reason="", speed=None, force=False):
    action = action.upper()
    if action not in {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"}:
        action = "STOP"
    speed = 0 if action == "STOP" else clamp_speed(MANUAL_DRIVE_SPEED if speed is None else speed)
    async with state_lock:
        if (not force and action == state["last_command"] and speed == state["last_speed"]
                and now() - state["last_command_at"] < COMMAND_MIN_INTERVAL_SEC):
            return
        state["last_command"] = action
        state["last_speed"] = speed
        state["last_command_at"] = now()
        state["last_reason"] = reason or state["last_reason"]
        print(f"[Drive] {action} (Speed: {speed}) | Reason: {reason}")
    if mqtt_client.is_connected():
        mqtt_client.publish(CONTROL_TOPIC, json.dumps({"drive": action, "speed": speed}), qos=1)
        state["mqtt_connected"] = True
    else:
        state["mqtt_connected"] = False


async def drive_heartbeat_loop():
    """Continuously sends keepalive drive packets to prevent ESP32-C3 motor timeout."""
    while True:
        await asyncio.sleep(0.25)
        async with state_lock:
            last_cmd = state["last_command"]
            last_spd = state["last_speed"]
            cmd_age = now() - state["last_command_at"]
        if last_cmd in {"FORWARD", "BACKWARD", "LEFT", "RIGHT"} and cmd_age < 1.5:
            if mqtt_client.is_connected():
                mqtt_client.publish(CONTROL_TOPIC, json.dumps({"drive": last_cmd, "speed": last_spd}), qos=1)


async def publish_dashboard_status():
    """Publish a retained state snapshot for MQTT dashboard clients."""
    if not mqtt_client.is_connected():
        return
    async with state_lock:
        snapshot = {**state, "telemetry": state["telemetry"].copy()}
    snapshot["telemetry"]["age_sec"] = telemetry_age()
    snapshot["target_age_sec"] = now() - state["last_target_seen_at"] if state["last_target_seen_at"] else None
    mqtt_client.publish(STATUS_TOPIC, json.dumps(snapshot), qos=0, retain=True)


def decide_follow_action(target, distance_mm, front_clearance_mm):
    global steering_direction

    yolo_detected = (
        target is not None
        and bool(target.get("visible"))
        and float(target.get("confidence") or 0) >= YOLO_CONFIDENCE
    )

    if yolo_detected:
        error_x = target["x"] - FRAME_CENTER_X
        # Steering priority: align horizontally with person first
        if error_x > TURN_ENTER_DEADBAND_PX or (steering_direction == "RIGHT" and error_x > TURN_EXIT_DEADBAND_PX):
            steering_direction = "RIGHT"
            return "RIGHT", AUTO_TURN_SPEED, f"YOLO: Person is {int(error_x)} px right. Turning right.", "YOLO"
        if error_x < -TURN_ENTER_DEADBAND_PX or (steering_direction == "LEFT" and error_x < -TURN_EXIT_DEADBAND_PX):
            steering_direction = "LEFT"
            return "LEFT", AUTO_TURN_SPEED, f"YOLO: Person is {abs(int(error_x))} px left. Turning left.", "YOLO"

        steering_direction = None

        # Person is centered in frame. Calculate visual distance from bounding box height.
        box = target.get("box")
        box_height = max(1.0, float(box[3] - box[1])) if box else 1.0
        # In 480p, at 650mm target follow distance, person's bounding box is ~340px
        visual_dist_mm = max(200.0, min(3500.0, (340.0 / box_height) * FOLLOW_TARGET_MM))

        # Ultrasonic distance measurement
        has_sonic = distance_mm is not None and valid_distance(distance_mm, 50, 3500)

        # Smart fusion: If ultrasonic sensor reading is trapped on a low ground/bumper bounce (<420mm)
        # while YOLO clearly sees person walking away (>850mm), follow the visual person!
        if has_sonic:
            if visual_dist_mm > (FOLLOW_TARGET_MM + 200) and distance_mm < (FOLLOW_MIN_MM + 100):
                effective_dist = visual_dist_mm
            else:
                effective_dist = distance_mm
        else:
            effective_dist = visual_dist_mm

        # Obstacle safety check: stop if an obstacle is dangerously close (< FOLLOW_MIN_MM)
        if has_sonic and distance_mm < FOLLOW_MIN_MM:
            return "STOP", 0, f"Safety stop: Obstacle close at {int(distance_mm)} mm.", "YOLO"

        if effective_dist < FOLLOW_MIN_MM:
            return "STOP", 0, f"YOLO: Person very close ({int(effective_dist)} mm). Holding.", "YOLO"
        if effective_dist > FOLLOW_TARGET_MM:
            spd = forward_speed(effective_dist)
            return "FORWARD", spd, f"YOLO: Following person at {int(effective_dist)} mm (Speed: {spd}).", "YOLO"
        return "STOP", 0, f"YOLO: Target in follow zone ({int(effective_dist)} mm). Holding.", "YOLO"

    # --- Sensor Tracking & Following (when YOLO box is temporarily absent) ---
    steering_direction = None

    if distance_mm is not None and valid_distance(distance_mm, 50, SENSOR_FOLLOW_MAX_MM):
        if distance_mm < FOLLOW_MIN_MM:
            return "STOP", 0, f"Sensor Tracking: Obstacle close at {int(distance_mm)} mm. Safety hold.", "SENSOR"

        if distance_mm > FOLLOW_TARGET_MM:
            spd = forward_speed(distance_mm)
            return "FORWARD", spd, f"Sensor Tracking: Following at {int(distance_mm)} mm (Speed: {spd}).", "SENSOR"

        return "STOP", 0, f"Sensor Tracking: Target in zone at {int(distance_mm)} mm. Holding.", "SENSOR"

    return "STOP", 0, "No target: YOLO searching & no sensor detection.", "NONE"


def open_rtsp_camera(target):
    global rtsp_cap, latest_frame, latest_frame_id
    if rtsp_cap is not None:
        rtsp_cap.release()
    url = target if str(target).startswith("rtsp://") else f"rtsp://{target}:554/"
    print(f"[Camera] Opening RTSP stream at {url}...")
    rtsp_cap = cv2.VideoCapture(url)
    rtsp_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    latest_frame = None
    latest_frame_id = 0
    is_open = rtsp_cap.isOpened()
    if is_open:
        print(f"[Camera] Successfully connected to {url}")
    else:
        print(f"[Camera] Failed to connect to {url}")
    return is_open


async def handle_task(task):
    task_type = str(task.get("task", "")).lower()
    async with state_lock:
        state["last_task_at"] = now()
    if task_type == "connect_camera":
        robot_ip = str(task.get("ip_address", "")).strip()
        connected = bool(robot_ip) and await asyncio.to_thread(open_rtsp_camera, robot_ip)
        async with state_lock:
            state["connected"] = connected
            state["last_reason"] = "Camera connected; YOLO processing is server-side." if connected else "Could not open RTSP camera."
    elif task_type == "set_target":
        reset_tracking()
        async with state_lock:
            state["target_profile"] = {"class": "person"}
            state["last_target"] = None
            state["last_target_seen_at"] = 0.0
            state["last_sensor_target_seen_at"] = 0.0
            state["tracking_source"] = "NONE"
            state["last_reason"] = "YOLO & sensor target set to person."
    elif task_type == "set_mode":
        mode = str(task.get("mode", "")).upper()
        if mode in {"AUTO", "MANUAL"}:
            async with state_lock:
                state["mode"] = mode
            await publish_drive("STOP", "Mode changed. Motors stopped.", force=True)
    elif task_type == "manual_drive" and state["mode"] == "MANUAL":
        await publish_drive(
            str(task.get("action", "STOP")),
            "Manual MQTT command.",
            speed=task.get("speed"),
            force=True,
        )
    elif task_type == "emergency_stop":
        async with state_lock:
            state["mode"] = "MANUAL"
        await publish_drive("STOP", "Emergency stop engaged.", force=True)


async def watchdog_loop():
    while True:
        await asyncio.sleep(0.2)
        if state["mode"] != "AUTO":
            continue
        yolo_age = now() - state["last_target_seen_at"] if state["last_target_seen_at"] else None
        sensor_age = now() - state["last_sensor_target_seen_at"] if state["last_sensor_target_seen_at"] else None
        tele_age = telemetry_age()
        tracking_src = state.get("tracking_source", "NONE")

        if tracking_src == "YOLO":
            if yolo_age is not None and yolo_age > TARGET_STALE_SEC:
                # If YOLO target went stale, check if sensor is currently tracking
                if tele_age is not None and tele_age <= TELEMETRY_STALE_SEC and sensor_age is not None and sensor_age <= TARGET_STALE_SEC:
                    pass
                else:
                    reset_tracking()
                    await publish_drive("STOP", "YOLO target lost & no sensor detection. Holding position.", force=True)
        elif tracking_src == "SENSOR":
            if tele_age is not None and tele_age > TELEMETRY_STALE_SEC:
                await publish_drive("STOP", "Sensor telemetry stale. Holding position.", force=True)
            elif sensor_age is not None and sensor_age > TARGET_STALE_SEC:
                await publish_drive("STOP", "Sensor target lost. Holding position.", force=True)
        elif tracking_src == "NONE":
            no_target_age = now() - state["last_command_at"]
            if state["last_command"] in {"FORWARD", "BACKWARD", "LEFT", "RIGHT"} and no_target_age > 0.8:
                await publish_drive("STOP", "No target detected. Holding position.", force=True)


async def dashboard_status_loop():
    while True:
        await publish_dashboard_status()
        await asyncio.sleep(0.5)


async def camera_capture_loop():
    global latest_frame, latest_frame_id
    consecutive_failures = 0
    while True:
        if rtsp_cap is None or not rtsp_cap.isOpened():
            camera_target = CAMERA_URL if CAMERA_URL else CAMERA_IP
            if camera_target:
                await asyncio.sleep(2.0)
                await asyncio.to_thread(open_rtsp_camera, camera_target)
            else:
                await asyncio.sleep(0.5)
            continue

        success, frame = await asyncio.to_thread(rtsp_cap.read)
        if success and frame is not None and frame.size > 0:
            latest_frame = frame
            latest_frame_id += 1
            consecutive_failures = 0
        else:
            consecutive_failures += 1
            if consecutive_failures > 30:
                print("[Camera] RTSP read dropped 30 frames consecutively. Reconnecting...")
                camera_target = CAMERA_URL if CAMERA_URL else CAMERA_IP
                await asyncio.to_thread(open_rtsp_camera, camera_target)
                consecutive_failures = 0
            await asyncio.sleep(0.03)


async def vision_loop():
    last_processed_frame_id = 0
    last_completed_at = now()
    while True:
        if latest_frame is None:
            # Camera frame is not available; allow sensor tracking fallback in AUTO mode
            if state["mode"] == "AUTO":
                await asyncio.sleep(0.1)
                telemetry = state["telemetry"]
                action, speed, reason, tracking_source = decide_follow_action(
                    None,
                    telemetry.get("sonic_mm"),
                    telemetry.get("front_clearance_mm"),
                )
                async with state_lock:
                    state["tracking_source"] = tracking_source
                    if tracking_source == "SENSOR":
                        state["last_sensor_target_seen_at"] = now()
                await publish_drive(action, reason, speed)
            else:
                await asyncio.sleep(0.1)
            continue

        if latest_frame_id == last_processed_frame_id or now() - state["last_ai_started_at"] < AI_INTERVAL_SEC:
            await asyncio.sleep(0.01)
            continue

        frame = latest_frame.copy()
        last_processed_frame_id = latest_frame_id
        state["last_ai_started_at"] = now()
        try:
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            target = await asyncio.to_thread(read_person_target, frame)
            telemetry = state["telemetry"]

            async with state_lock:
                state["last_target"] = target
                state["vision_fps"] = round(1 / max(now() - last_completed_at, 0.001), 1)
                state["inference_ms"] = round((now() - state["last_ai_started_at"]) * 1000, 1)
                if target.get("visible"):
                    state["last_target_seen_at"] = now()

            if state["mode"] == "AUTO":
                action, speed, reason, tracking_source = decide_follow_action(
                    target,
                    telemetry.get("sonic_mm"),
                    telemetry.get("front_clearance_mm"),
                )
                async with state_lock:
                    state["tracking_source"] = tracking_source
                    if tracking_source == "YOLO":
                        state["last_target_seen_at"] = now()
                    elif tracking_source == "SENSOR":
                        state["last_sensor_target_seen_at"] = now()
                await publish_drive(action, reason, speed)
            else:
                async with state_lock:
                    state["tracking_source"] = "YOLO" if target.get("visible") else "NONE"

            last_completed_at = now()
        except Exception as exc:
            print(f"YOLO vision error: {exc}")
            if state["mode"] == "AUTO":
                await publish_drive("STOP", "YOLO vision error. Holding position.", force=True)
            await asyncio.sleep(0.05)


async def display_loop():
    """Live video stream GUI with real-time YOLO bounding box and status overlay."""
    if not SHOW_STREAM:
        print("[GUI] Video stream display window disabled (SHOW_STREAM=0).")
        return

    print(f"[GUI] Launching video stream display window: '{WINDOW_TITLE}'...")
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_TITLE, FRAME_WIDTH, FRAME_HEIGHT)

    try:
        while stop_event is not None and not stop_event.is_set():
            if latest_frame is not None:
                frame_to_show = latest_frame.copy()
                if frame_to_show.shape[1] != FRAME_WIDTH or frame_to_show.shape[0] != FRAME_HEIGHT:
                    frame_to_show = cv2.resize(frame_to_show, (FRAME_WIDTH, FRAME_HEIGHT))
                async with state_lock:
                    target = state["last_target"]
                    telemetry = state["telemetry"].copy()
                    state_snapshot = state.copy()
                draw_hud(frame_to_show, target, telemetry, state_snapshot)
            else:
                frame_to_show = create_placeholder_frame()

            cv2.imshow(WINDOW_TITLE, frame_to_show)

            # Detect window close event via window property
            if cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                print("[GUI] Video stream window closed by user. Stopping server...")
                stop_event.set()
                break

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):  # 'q' or ESC
                print("[GUI] Exit key pressed. Stopping server...")
                stop_event.set()
                break
            elif key in (ord("m"), ord("M")):
                async with state_lock:
                    new_mode = "MANUAL" if state["mode"] == "AUTO" else "AUTO"
                    state["mode"] = new_mode
                if new_mode == "MANUAL":
                    await publish_drive("STOP", "Switched to MANUAL mode via GUI key.", force=True)
                else:
                    await publish_drive("STOP", "Switched to AUTO mode via GUI key.", force=True)
                print(f"[GUI] Mode switched to: {new_mode}")
            elif key in (ord("w"), ord("W")):
                if state["mode"] == "MANUAL":
                    await publish_drive("FORWARD", "Manual drive W", MANUAL_DRIVE_SPEED, force=True)
            elif key in (ord("s"), ord("S")):
                if state["mode"] == "MANUAL":
                    await publish_drive("BACKWARD", "Manual drive S", MANUAL_DRIVE_SPEED, force=True)
                else:
                    await publish_drive("STOP", "Emergency STOP via GUI key.", force=True)
            elif key in (ord("a"), ord("A")):
                if state["mode"] == "MANUAL":
                    await publish_drive("LEFT", "Manual drive A", AUTO_TURN_SPEED, force=True)
            elif key in (ord("d"), ord("D")):
                if state["mode"] == "MANUAL":
                    await publish_drive("RIGHT", "Manual drive D", AUTO_TURN_SPEED, force=True)
            elif key in (ord(" "), ord("x"), ord("X")):  # Space or 'x'
                await publish_drive("STOP", "Manual STOP via GUI key.", force=True)

            await asyncio.sleep(0.015)
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        print(f"[GUI] Display loop exception: {exc}")
    finally:
        cv2.destroyAllWindows()


async def run_server():
    """Run the MQTT, RTSP capture, vision, and video display workers."""
    global server_loop, stop_event
    server_loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    print(f"[Server] Starting Follower Robot Server (Robot ID: {ROBOT_ID})")
    print(f"[MQTT] Connecting to broker {MQTT_BROKER}:{MQTT_PORT}...")
    connect_mqtt()

    camera_target = CAMERA_URL if CAMERA_URL else CAMERA_IP
    if camera_target:
        connected = await asyncio.to_thread(open_rtsp_camera, camera_target)
        async with state_lock:
            state["connected"] = connected
            state["last_reason"] = (
                "Camera connected; YOLO processing is server-side."
                if connected
                else f"Could not open RTSP camera at {camera_target}."
            )

    watchdog_task = asyncio.create_task(watchdog_loop())
    dashboard_status_task = asyncio.create_task(dashboard_status_loop())
    heartbeat_task = asyncio.create_task(drive_heartbeat_loop())
    camera_task = asyncio.create_task(camera_capture_loop())
    vision_task = asyncio.create_task(vision_loop())
    display_task = asyncio.create_task(display_loop())

    try:
        await stop_event.wait()
    finally:
        print("[Server] Shutting down workers...")
        tasks = (watchdog_task, dashboard_status_task, heartbeat_task, camera_task, vision_task, display_task)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        try:
            await publish_drive("STOP", "Server shutdown. Motors halted.", force=True)
        except Exception:
            pass
        cv2.destroyAllWindows()
        if rtsp_cap is not None:
            rtsp_cap.release()
        if mqtt_loop_started:
            mqtt_client.loop_stop()
        print("[Server] Shutdown complete.")


if __name__ == "__main__":
    try:
        asyncio.run(run_server())
    except KeyboardInterrupt:
        print("\n[Server] Interrupted by user (Ctrl+C). Exiting.")
