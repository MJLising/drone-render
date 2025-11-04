# app.py
import os, json, time
from fastapi import FastAPI, Request, HTTPException, Form
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt

APP_PORT = int(os.getenv("PORT", "8000"))
MQTT_BROKER = os.getenv("MQTT_BROKER", "broker.hivemq.com")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
# topic should be unique per device/pi (use device id)
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "drone/controls/mydevice")
DRONE_AUTH_TOKEN = os.getenv("DRONE_AUTH_TOKEN", "")  # shared secret

app = FastAPI()
templates = Jinja2Templates(directory="templates")

# serve static controls page
@app.get("/", response_class=HTMLResponse)
def index(request: Request):
    return templates.TemplateResponse("controls.html", {"request": request, "token": DRONE_AUTH_TOKEN})

# publish JSON to MQTT. Token is required in the posted payload.
@app.post("/api/command")
async def api_command(payload: dict = None, token: str = Form(None)):
    # Payload may come as JSON form or via JSON body: form field 'token' or JSON 'token' accepted
    body = payload or {}
    # token check (first try body then form param)
    tkn = body.get("token") if body.get("token") else token
    if DRONE_AUTH_TOKEN:
        if not tkn or tkn != DRONE_AUTH_TOKEN:
            raise HTTPException(status_code=401, detail="missing/invalid token")
    # attach server timestamp
    body["_ts"] = int(time.time())
    payload_bytes = json.dumps(body).encode("utf-8")
    # publish (no blocking broker auth if not provided)
    auth = None
    if MQTT_USERNAME and MQTT_PASSWORD:
        auth = {'username': MQTT_USERNAME, 'password': MQTT_PASSWORD}
    try:
        publish.single(MQTT_TOPIC, payload_bytes, hostname=MQTT_BROKER, port=MQTT_PORT, auth=auth, keepalive=30)
    except Exception as e:
        raise HTTPException(status_code=502, detail=f"MQTT publish failed: {e}")
    return {"published_to": MQTT_TOPIC, "payload": body}

# health endpoint
@app.get("/health")
def health():
    return {"status": "ok"}

# optionally run with uvicorn on Render; Render populates $PORT automatically
