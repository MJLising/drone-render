# app.py  (Render-side, public)
import os
import json
import asyncio
import base64
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import paho.mqtt.client as mqtt
from typing import Set

APP = FastAPI()
BASE_DIR = os.path.dirname(__file__)
CONTROLS_PATH = os.path.join(BASE_DIR, "controls.html")

MQTT_BROKER = os.getenv("MQTT_BROKER", "")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")
DRONE_ID = os.getenv("DRONE_ID", "drone1")

TOPIC_CMD = f"drone/{DRONE_ID}/cmd"
TOPIC_TELE = f"drone/{DRONE_ID}/telemetry"
TOPIC_THUMB = f"drone/{DRONE_ID}/thumb"

# connected websockets
clients: Set[WebSocket] = set()
loop = asyncio.get_event_loop()

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    print("[MQTT] connected rc=", rc)
    client.subscribe(TOPIC_TELE)
    client.subscribe(TOPIC_THUMB)

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload
    # forward to websockets
    data = None
    if topic == TOPIC_TELE:
        try:
            data = {"type": "telemetry", "data": json.loads(payload.decode('utf-8'))}
        except Exception:
            data = {"type":"telemetry","data":payload.decode('utf-8', errors='ignore')}
    elif topic == TOPIC_THUMB:
        try:
            b64 = payload.decode('ascii')
            data = {"type":"thumb","data": b64}
        except Exception:
            data = {"type":"thumb","data": None}
    else:
        data = {"type":"unknown","topic":topic,"data":payload.decode('utf-8',errors='ignore')}
    # schedule send to websockets
    coro = broadcast(json.dumps(data))
    try:
        asyncio.run_coroutine_threadsafe(coro, loop)
    except Exception as e:
        print("[MQTT] broadcast schedule error:", e)

# start mqtt client in background thread
mqtt_client = mqtt.Client()
if MQTT_USER:
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS or None)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    mqtt_client.loop_start()
    print("[MQTT] started client")
except Exception as e:
    print("[MQTT] connect error:", e)

@APP.get("/")
def index():
    if os.path.exists(CONTROLS_PATH):
        return FileResponse(CONTROLS_PATH, media_type="text/html")
    return {"status":"controls.html missing"}

# websocket endpoint for browser UI
@APP.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        await ws.send_text(json.dumps({"type":"info","data":"connected to relay"}))
        while True:
            txt = await ws.receive_text()
            try:
                payload = json.loads(txt)
            except Exception:
                payload = {"_cmd": txt}
            # publish to mqtt
            try:
                mqtt_client.publish(TOPIC_CMD, json.dumps(payload))
            except Exception as e:
                print("[WS] mqtt publish error:", e)
            # optionally echo back
            await ws.send_text(json.dumps({"type":"ack","data":payload}))
    except WebSocketDisconnect:
        clients.discard(ws)

async def broadcast(msg: str):
    # send to all connected websockets
    to_remove = []
    for c in list(clients):
        try:
            await c.send_text(msg)
        except Exception:
            to_remove.append(c)
    for c in to_remove:
        clients.discard(c)
