import os
import json
import asyncio
import cv2
import base64
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, StreamingResponse
import paho.mqtt.client as mqtt
from groq import Groq

app = FastAPI()
groq_client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

# --- HiveMQ & Routing Configurations ---
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
ROBOT_ID = "sentinel_alpha_99x2" 
CONTROL_TOPIC = f"nodes/{ROBOT_ID}/hardware_control"
TELEMETRY_TOPIC = f"nodes/{ROBOT_ID}/telemetry"

current_target_profile = {"attributes": "human"}
frame_center_x = 320  
rtsp_cap = None
frame_counter = 0

# Fused global distance parameter (in millimeters)
fused_distance_mm = 0.0

# System State: "AUTO" or "MANUAL"
system_mode = "AUTO"

# --- Sensor Fusion Decision Handler ---
def on_message(client, userdata, msg):
    global fused_distance_mm
    try:
        if msg.topic == TELEMETRY_TOPIC:
            payload = json.loads(msg.payload.decode())
            laser = float(payload.get("laser_mm", -1))
            sonic = float(payload.get("sonic_mm", 9999))
            
            if 20 < laser < 1200:
                fused_distance_mm = laser
            elif 20 < sonic < 3000:
                fused_distance_mm = sonic
            else:
                fused_distance_mm = 0.0 
    except Exception as e:
        print(f"Sensor Fusion parsing error: {e}")

# Initialize MQTT Client
mqtt_client = mqtt.Client(client_id=f"GroqServerAgent_{ROBOT_ID}")
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.subscribe(TELEMETRY_TOPIC)
mqtt_client.loop_start()

@app.post("/connect-rtsp")
async def connect_rtsp(payload: dict):
    global rtsp_cap
    robot_ip = payload.get("ip_address")
    if rtsp_cap is not None: 
        rtsp_cap.release()
    rtsp_cap = cv2.VideoCapture(f"rtsp://{robot_ip}:554/")
    await asyncio.sleep(1.0)
    return {"status": "connected" if rtsp_cap.isOpened() else "error"}

@app.post("/set-target")
async def set_target(payload: dict):
    global current_target_profile
    prompt = f"Extract distinct visual signatures into raw flat JSON from: '{payload.get('description')}'"
    response = groq_client.chat.completions.create(
        model="llama3-70b-8192", 
        messages=[{"role": "user", "content": prompt}], 
        response_format={"type": "json_object"}
    )
    current_target_profile = json.loads(response.choices[0].message.content)
    return {"status": "success", "profile": current_target_profile}

@app.post("/emergency-stop")
async def emergency_stop():
    global system_mode
    system_mode = "MANUAL" # Drop back to manual safety on E-stop
    mqtt_client.publish(CONTROL_TOPIC, json.dumps({"drive": "STOP"}), qos=2)
    return {"status": "HALTED", "mode": system_mode}

# --- New: Mode Selector Route ---
@app.post("/set-mode")
async def set_mode(payload: dict):
    global system_mode
    requested_mode = payload.get("mode", "AUTO").upper()
    if requested_mode in ["AUTO", "MANUAL"]:
        system_mode = requested_mode
        # Cut motor speeds to 0 immediately during transitions
        mqtt_client.publish(CONTROL_TOPIC, json.dumps({"drive": "STOP"}))
        return {"status": "success", "mode": system_mode}
    return {"status": "error", "message": "Invalid mode specified"}

# --- New: Manual Control Route ---
@app.post("/manual-drive")
async def manual_drive(payload: dict):
    global system_mode
    if system_mode != "MANUAL":
        return {"status": "error", "message": "Enable Manual Mode first"}
    
    action = payload.get("action", "STOP").upper()
    if action in ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"]:
        mqtt_client.publish(CONTROL_TOPIC, json.dumps({"drive": action}))
        return {"status": "success", "action": action}
    return {"status": "error", "message": "Invalid direction"}

async def generate_frames():
    global rtsp_cap, frame_counter
    while True:
        if rtsp_cap is None or not rtsp_cap.isOpened():
            await asyncio.sleep(0.1)
            continue
        success, frame = rtsp_cap.read()
        if not success: 
            await asyncio.sleep(0.01)
            continue
        
        frame_counter += 1
        # Only analyze camera frames if AI tracking mode is explicitly enabled
        if frame_counter % 6 == 0 and system_mode == "AUTO":
            asyncio.create_task(run_agentic_pipeline(frame.copy()))
            
        ret, jpeg = cv2.imencode('.jpg', frame)
        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
        await asyncio.sleep(0.01)

# --- Vision AI Pipeline (Unchanged, now respects AUTO state flag) ---
async def run_agentic_pipeline(img):
    global current_target_profile, fused_distance_mm
    try:
        resized = cv2.resize(img, (640, 480))
        _, buffer = cv2.imencode('.jpg', resized)
        base64_image = base64.b64encode(buffer).decode('utf-8')
        
        response = groq_client.chat.completions.create(
            model="llama3-groq-8b-8192-tool-use-preview",
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": f"Locate target: {json.dumps(current_target_profile)}. Return center coordinates relative to 640x480 box as strict JSON: {{\"x\": int, \"y\": int}}"},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                ]
            }],
            response_format={"type": "json_object"}
        )
        
        coordinates = json.loads(response.choices[0].message.content)
        tx = coordinates.get("x")
        
        if tx is not None and system_mode == "AUTO": # Final safety gate
            error_x = tx - frame_center_x
            motor_action = "STOP"
            
            if error_x > 75:
                motor_action = "RIGHT"
            elif error_x < -75:
                motor_action = "LEFT"
            else:
                if fused_distance_mm > 450:
                    motor_action = "FORWARD"
                elif 0 < fused_distance_mm < 280:
                    motor_action = "BACKWARD"
                else:
                    motor_action = "STOP"
            
            control_payload = {"drive": motor_action}
            mqtt_client.publish(CONTROL_TOPIC, json.dumps(control_payload))
            
    except Exception as e:
        print(f"Agent Engine Error: {e}")

@app.get("/video-feed")
async def video_feed_endpoint():
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/", response_class=HTMLResponse)
async def index():
    with open("index.html", "r") as f: 
        return f.read()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
