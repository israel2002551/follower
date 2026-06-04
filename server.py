import os
import json
import asyncio
import cv2
import base64
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaStreamTrack
import paho.mqtt.client as mqtt
from groq import Groq

# Initialization
app = FastAPI()
groq_client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

# HiveMQ Public Broker Configuration
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883

# Isolated namespace to prevent third-party collisions on HiveMQ
ROBOT_ID = "sentinel_alpha_99x2" 
CONTROL_TOPIC = f"nodes/{ROBOT_ID}/hardware_control"

# MQTT Client Setup
mqtt_client = mqtt.Client(client_id=f"GroqServerAgent_{ROBOT_ID}")
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# Global Agent Tracking Parameters
current_target_profile = {"attributes": "human"}
frame_center_x = 320  
frame_center_y = 240

# --- AGENT 1: The Orchestrator Agent ---
@app.post("/set-target")
async def set_target(payload: dict):
    global current_target_profile
    user_description = payload.get("description", "a person")
    
    prompt = (
        "You are an elite robotic tracking orchestrator. Analyze this natural language description "
        f"and extract distinct visual feature signatures: '{user_description}'. "
        "Output your response strictly as a flat JSON structure containing structural features like clothing_color, "
        "hair, accessories, or unique markings. Do not include any conversational text."
    )
    
    response = groq_client.chat.completions.create(
        model="llama3-70b-8192",
        messages=[{"role": "user", "content": prompt}],
        response_format={"type": "json_object"}
    )
    
    current_target_profile = json.loads(response.choices[0].message.content)
    return {"status": "success", "profile": current_target_profile}

# --- HIGH PRIORITY SAFETY SAFETY ROUTE: Emergency System Halt ---
@app.post("/emergency-stop")
async def emergency_stop():
    stop_payload = {
        "pan": 0,
        "drive": "STOP"
    }
    # Publish instant override packet across HiveMQ network with QoS 2 (Assured Delivery)
    mqtt_client.publish(CONTROL_TOPIC, json.dumps(stop_payload), qos=2)
    return {"status": "HALTED"}

# --- AGENT 2 & 3: Vision Perceiver & Spatial Action Stack ---
class RobotVisionProcessor(MediaStreamTrack):
    kind = "video"

    def __init__(self, track):
        super().__init__()
        self.track = track

    async def recv(self):
        frame = await self.track.recv()
        img = frame.to_ndarray(format="bgr24")
        
        # Sub-sample frames to manage Groq API token speed thresholds
        if frame.pts % 5 == 0:
            asyncio.create_task(self.run_agentic_vision_pipeline(img))
            
        return frame

    async def run_agentic_vision_pipeline(self, img):
        global current_target_profile
        try:
            resized = cv2.resize(img, (640, 480))
            _, buffer = cv2.imencode('.jpg', resized)
            base64_image = base64.b64encode(buffer).decode('utf-8')
            
            response = groq_client.chat.completions.create(
                model="llama3-groq-8b-8192-tool-use-preview",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": f"Locate the item matching this profile: {json.dumps(current_target_profile)}. Return only its bounding box center X and Y integers relative to a 640x480 resolution as JSON format: {{\"x\": int, \"y\": int, \"box_width\": int}}"},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                        ]
                    }
                ],
                response_format={"type": "json_object"}
            )
            
            coordinates = json.loads(response.choices[0].message.content)
            target_x = coordinates.get("x")
            target_y = coordinates.get("y")
            box_w = coordinates.get("box_width", 100)
            
            if target_x is not None and target_y is not None:
                self.calculate_and_publish_actions(target_x, target_y, box_w)
                
        except Exception as e:
            print(f"Agent Pipeline Error: {e}")

    def calculate_and_publish_actions(self, tx, ty, tw):
        error_x = tx - frame_center_x
        
        pan_command = 0
        if abs(error_x) > 30:
            pan_command = -5 if error_x > 0 else 5  
            
        motor_action = "STOP"
        if tw < 130:     
            motor_action = "FORWARD"
        elif tw > 210:   
            motor_action = "BACKWARD"
        elif error_x > 75:
            motor_action = "RIGHT"
        elif error_x < -75:
            motor_action = "LEFT"
            
        control_payload = {
            "pan": pan_command,
            "drive": motor_action
        }
        mqtt_client.publish(CONTROL_TOPIC, json.dumps(control_payload))

@app.post("/webrtc-negotiate")
async def webrtc_negotiate(params: dict):
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    pc = RTCPeerConnection()
    
    @pc.on("track")
    def on_track(track):
        if track.kind == "video":
            local_video_processor = RobotVisionProcessor(track)
            pc.addTrack(local_video_processor)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

@app.get("/", response_class=HTMLResponse)
async def index():
    with open("index.html", "r") as f:
        return f.read()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
