import asyncio
import json
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path

import cv2
import paho.mqtt.client as mqtt
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from ultralytics import YOLO


ROBOT_ID = os.getenv("ROBOT_ID", "sentinel_alpha_99x2")
MQTT_BROKER = os.getenv("MQTT_BROKER", "broker.emqx.io")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_WS_URL = os.getenv("MQTT_WS_URL", "wss://broker.emqx.io:8084/mqtt")
CONTROL_TOPIC = f"nodes/{ROBOT_ID}/hardware_control"
TELEMETRY_TOPIC = f"nodes/{ROBOT_ID}/telemetry"
TASK_TOPIC = f"nodes/{ROBOT_ID}/tasks"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2
TURN_ENTER_DEADBAND_PX = 80
TURN_EXIT_DEADBAND_PX = 45
FOLLOW_MIN_MM = 320
FOLLOW_TARGET_MM = 520
FOLLOW_MAX_MM = 850
AUTO_MIN_SPEED = int(os.getenv("AUTO_MIN_SPEED", "105"))
AUTO_MAX_SPEED = int(os.getenv("AUTO_MAX_SPEED", "185"))
AUTO_TURN_SPEED = int(os.getenv("AUTO_TURN_SPEED", "125"))
MANUAL_DRIVE_SPEED = int(os.getenv("MANUAL_DRIVE_SPEED", "170"))
TELEMETRY_STALE_SEC = 2.0
TARGET_STALE_SEC = 1.8
AI_INTERVAL_SEC = 0.08
COMMAND_MIN_INTERVAL_SEC = 0.18
YOLO_MODEL_PATH = os.getenv("YOLO_MODEL", "yolo11n.pt")
YOLO_CONFIDENCE = float(os.getenv("YOLO_CONFIDENCE", "0.45"))
YOLO_IMAGE_SIZE = int(os.getenv("YOLO_IMAGE_SIZE", "320"))
TARGET_SMOOTHING = float(os.getenv("TARGET_SMOOTHING", "0.55"))

rtsp_cap = None
yolo_model = None
mqtt_loop_started = False
server_loop = None
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
    "last_ai_started_at": 0.0,
    "last_command": "STOP",
    "last_speed": 0,
    "last_command_at": 0.0,
    "last_reason": "Waiting for camera connection.",
    "mqtt_connected": False,
    "last_task_at": 0.0,
    "vision_fps": 0.0,
    "inference_ms": None,
    "telemetry": {"laser_mm": None, "sonic_mm": None, "fused_distance_mm": None, "received_at": 0.0},
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


def fuse_sensor_distances(laser_mm, sonic_mm):
    laser = float(laser_mm) if valid_distance(laser_mm, 20, 1200) else None
    sonic = float(sonic_mm) if valid_distance(sonic_mm, 20, 3000) else None
    readings = [reading for reading in (laser, sonic) if reading is not None]
    if not readings:
        return None, None

    front_clearance = min(readings)
    if laser is None:
        return sonic, front_clearance
    if sonic is None:
        return laser, front_clearance
    if abs(laser - sonic) <= 180:
        return round((laser * 0.7) + (sonic * 0.3), 1), front_clearance
    return laser, front_clearance


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
        return {"visible": False, "x": None, "y": None, "confidence": 0.0, "box": None}

    candidates = []
    for box, confidence in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist()):
        x1, y1, x2, y2 = [float(value) for value in box]
        candidates.append({
            "box": (x1, y1, x2, y2),
            "x": (x1 + x2) / 2,
            "y": (y1 + y2) / 2,
            "area": max(1.0, (x2 - x1) * (y2 - y1)),
            "confidence": round(float(confidence), 3),
        })
    return select_target(candidates)


def on_message(client, userdata, msg):
    if msg.topic == TELEMETRY_TOPIC:
        try:
            payload = json.loads(msg.payload.decode())
            laser = payload.get("laser_mm")
            sonic = payload.get("sonic_mm")
            fused, front_clearance = fuse_sensor_distances(laser, sonic)
            state["telemetry"] = {
                "laser_mm": laser if valid_distance(laser, 20, 4000) else None,
                "sonic_mm": sonic if valid_distance(sonic, 20, 5000) else None,
                "fused_distance_mm": fused,
                "front_clearance_mm": front_clearance,
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


try:
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"FollowerServer_{ROBOT_ID}")
except AttributeError:
    mqtt_client = mqtt.Client(client_id=f"FollowerServer_{ROBOT_ID}")
mqtt_client.on_message = on_message


def connect_mqtt():
    global mqtt_loop_started
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.subscribe([(TELEMETRY_TOPIC, 0), (TASK_TOPIC, 1)])
        if not mqtt_loop_started:
            mqtt_client.loop_start()
            mqtt_loop_started = True
        state["mqtt_connected"] = True
        return True
    except Exception as exc:
        state["mqtt_connected"] = False
        state["last_reason"] = f"MQTT broker unavailable: {exc}"
        print(state["last_reason"])
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
    if not mqtt_client.is_connected():
        connect_mqtt()
    if mqtt_client.is_connected():
        mqtt_client.publish(CONTROL_TOPIC, json.dumps({"drive": action, "speed": speed}), qos=1)
        state["mqtt_connected"] = True
    else:
        state["mqtt_connected"] = False


def decide_follow_action(target, distance_mm, front_clearance_mm):
    global steering_direction
    if not target or not target.get("visible"):
        steering_direction = None
        return "STOP", 0, "Person not visible."
    if float(target.get("confidence") or 0) < YOLO_CONFIDENCE:
        return "STOP", 0, "Low YOLO confidence."
    error_x = target["x"] - FRAME_CENTER_X
    if error_x > TURN_ENTER_DEADBAND_PX or (steering_direction == "RIGHT" and error_x > TURN_EXIT_DEADBAND_PX):
        steering_direction = "RIGHT"
        return "RIGHT", AUTO_TURN_SPEED, f"Person is {int(error_x)} px right of center."
    if error_x < -TURN_ENTER_DEADBAND_PX or (steering_direction == "LEFT" and error_x < -TURN_EXIT_DEADBAND_PX):
        steering_direction = "LEFT"
        return "LEFT", AUTO_TURN_SPEED, f"Person is {abs(int(error_x))} px left of center."
    steering_direction = None
    if distance_mm is None or front_clearance_mm is None:
        return "STOP", 0, "Waiting for valid distance telemetry."
    if front_clearance_mm < FOLLOW_MIN_MM:
        return "STOP", 0, f"Front clearance is unsafe at {int(front_clearance_mm)} mm."
    if distance_mm > FOLLOW_MAX_MM:
        return "FORWARD", forward_speed(distance_mm), f"Person is far at {int(distance_mm)} mm; front path is clear."
    if distance_mm < FOLLOW_MIN_MM:
        return "STOP", 0, f"Person is close at {int(distance_mm)} mm; holding without rear sensing."
    if distance_mm > FOLLOW_TARGET_MM:
        return "FORWARD", forward_speed(distance_mm), f"Closing distance at {int(distance_mm)} mm."
    return "STOP", 0, f"Person held at {int(distance_mm)} mm."


def open_rtsp_camera(robot_ip):
    global rtsp_cap, latest_frame, latest_frame_id
    if rtsp_cap is not None:
        rtsp_cap.release()
    rtsp_cap = cv2.VideoCapture(f"rtsp://{robot_ip}:554/")
    rtsp_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    latest_frame = None
    latest_frame_id = 0
    return rtsp_cap.isOpened()


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
            state["last_reason"] = "YOLO target set to person."
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
        target_age = now() - state["last_target_seen_at"] if state["last_target_seen_at"] else None
        tele_age = telemetry_age()
        if target_age is not None and target_age > TARGET_STALE_SEC:
            reset_tracking()
            await publish_drive("STOP", "Person lost. Holding position.", force=True)
        elif tele_age is not None and tele_age > TELEMETRY_STALE_SEC and state["last_command"] in {"FORWARD", "BACKWARD"}:
            await publish_drive("STOP", "Distance telemetry stale. Holding position.", force=True)


async def camera_capture_loop():
    global latest_frame, latest_frame_id
    while True:
        if rtsp_cap is None or not rtsp_cap.isOpened():
            await asyncio.sleep(0.1)
            continue
        success, frame = await asyncio.to_thread(rtsp_cap.read)
        if success:
            latest_frame = frame
            latest_frame_id += 1
        else:
            await asyncio.sleep(0.03)


async def vision_loop():
    last_processed_frame_id = 0
    last_completed_at = now()
    while True:
        if state["mode"] != "AUTO" or latest_frame is None:
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
            action, speed, reason = decide_follow_action(
                target,
                telemetry.get("fused_distance_mm"),
                telemetry.get("front_clearance_mm"),
            )
            async with state_lock:
                state["last_target"] = target
                state["vision_fps"] = round(1 / max(now() - last_completed_at, 0.001), 1)
                state["inference_ms"] = round((now() - state["last_ai_started_at"]) * 1000, 1)
                if target["visible"]:
                    state["last_target_seen_at"] = now()
            last_completed_at = now()
            await publish_drive(action, reason, speed)
        except Exception as exc:
            print(f"YOLO vision error: {exc}")
            await publish_drive("STOP", "YOLO vision error. Holding position.", force=True)


@asynccontextmanager
async def lifespan(app_instance):
    global server_loop
    server_loop = asyncio.get_running_loop()
    connect_mqtt()
    watchdog_task = asyncio.create_task(watchdog_loop())
    camera_task = asyncio.create_task(camera_capture_loop())
    vision_task = asyncio.create_task(vision_loop())
    try:
        yield
    finally:
        watchdog_task.cancel()
        camera_task.cancel()
        vision_task.cancel()
        if rtsp_cap is not None:
            rtsp_cap.release()
        if mqtt_loop_started:
            mqtt_client.loop_stop()


app = FastAPI(lifespan=lifespan)


@app.get("/status")
async def status():
    telemetry = state["telemetry"].copy()
    telemetry["age_sec"] = telemetry_age()
    return {**state, "telemetry": telemetry, "target_age_sec": now() - state["last_target_seen_at"] if state["last_target_seen_at"] else None}


@app.get("/", response_class=HTMLResponse)
async def index():
    return (Path("index.html").read_text(encoding="utf-8")
            .replace("__MQTT_WS_URL__", MQTT_WS_URL)
            .replace("__ROBOT_ID__", ROBOT_ID))


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
