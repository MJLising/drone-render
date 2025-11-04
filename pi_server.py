#!/usr/bin/env python3
"""
pi_server.py - Pi-side server (camera + TFLite overlay + motor control + MJPEG + local WS/REST)
Adds MQTT client to receive remote commands and publish telemetry.

Env used:
 - DRONE_AUTH_TOKEN       optional token for control endpoints & ws
 - PREFERRED_VIDEO_DEVICE optional device path e.g. /dev/video0
 - STREAM_W, STREAM_H     output MJPEG size
 - JPEG_QUALITY           JPEG encoding quality
 - PI_SERVER_PORT         port (default 8000)
 - MQTT_BROKER            hostname (required for remote control) e.g. broker.hivemq.cloud
 - MQTT_PORT              broker port (default 1883 or 8883)
 - MQTT_USER              optional username
 - MQTT_PASS              optional password
 - DRONE_ID               unique ID for the drone (topic namespace), default "drone1"
 - PUBLISH_THUMB          "1" to publish small base64 JPG thumbnails to telemetry (optional)
"""
import os
import time
import json
import glob
import threading
import base64
from typing import Optional
from pathlib import Path

from fastapi import FastAPI, Request, Depends, HTTPException, WebSocket, WebSocketDisconnect, Query
from fastapi.responses import StreamingResponse, FileResponse
import cv2
import numpy as np

# motor abstraction (unchanged)
from drone_motors import set_motor, stop_all

# MQTT client
try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None

# --- config ---
APP_HOST = "0.0.0.0"
APP_PORT = int(os.getenv("PI_SERVER_PORT", "8000"))
STREAM_W = int(os.getenv("STREAM_W", "640"))
STREAM_H = int(os.getenv("STREAM_H", "360"))
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "60"))
ENCODE_INTERVAL = float(os.getenv("ENCODE_INTERVAL", "0.04"))
AUTH_TOKEN = os.getenv("DRONE_AUTH_TOKEN", "").strip()
PREFERRED_DEVICE = os.getenv("PREFERRED_VIDEO_DEVICE", "").strip()
DEFAULT_SPEED = int(os.getenv("DEFAULT_SPEED", "160"))

# MQTT
MQTT_BROKER = os.getenv("MQTT_BROKER", "").strip()
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "").strip()
MQTT_PASS = os.getenv("MQTT_PASS", "").strip()
DRONE_ID = os.getenv("DRONE_ID", "drone1")
PUBLISH_THUMB = os.getenv("PUBLISH_THUMB", "0") == "1"

TOPIC_CMD = f"drone/{DRONE_ID}/cmd"
TOPIC_TELE = f"drone/{DRONE_ID}/telemetry"
TOPIC_THUMB = f"drone/{DRONE_ID}/thumb"

app = FastAPI(title="Pi Drone Server (local)")

# --- TFLite helper (unchanged) ---
def make_interpreter(model_path):
    try:
        from tflite_runtime.interpreter import Interpreter
        return Interpreter(model_path=model_path)
    except Exception:
        try:
            from tensorflow.lite.python.interpreter import Interpreter
            return Interpreter(model_path=model_path)
        except Exception:
            return None

# --- Camera + AI thread (kept from your working script, mostly unchanged) ---
class CameraThread(threading.Thread):
    def __init__(self, cam_index=0, model_path=None, labels_path=None, out_w=640, out_h=360):
        super().__init__(daemon=True)
        self.cam_index = cam_index
        self.model_path = model_path
        self.labels_path = labels_path
        self.out_w = out_w
        self.out_h = out_h

        self.stop_event = threading.Event()
        self.frame_lock = threading.Lock()
        self.latest_frame = None

        # tflite
        self.interpreter = None
        self.input_details = None
        self.output_details = None
        self.model_width = None
        self.model_height = None
        self.floating_model = False
        self.input_mean = 127.5
        self.input_std = 127.5
        self.labels = []
        self._frame_counter = 0
        self.INFERENCE_EVERY_N = 8
        self._last_label = None

        # load labels
        if self.labels_path and Path(self.labels_path).exists():
            try:
                with open(self.labels_path, "r", encoding="utf-8") as f:
                    self.labels = [ln.strip() for ln in f if ln.strip()]
                print(f"[CameraThread] Loaded {len(self.labels)} labels.")
            except Exception as e:
                print("[CameraThread] label read error:", e)

        # load model
        if self.model_path and Path(self.model_path).exists():
            print("[CameraThread] Attempting to load TFLite model:", self.model_path)
            interp = make_interpreter(self.model_path)
            if interp is None:
                print("[CameraThread] tflite runtime unavailable; running camera-only.")
                self.interpreter = None
            else:
                try:
                    self.interpreter = interp
                    self.interpreter.allocate_tensors()
                    self.input_details = self.interpreter.get_input_details()
                    self.output_details = self.interpreter.get_output_details()
                    self.model_height = self.input_details[0]['shape'][1]
                    self.model_width  = self.input_details[0]['shape'][2]
                    self.floating_model = self.input_details[0]['dtype'] == np.float32
                    print(f"[CameraThread] Model loaded input {self.model_width}x{self.model_height} float={self.floating_model}")
                except Exception as e:
                    print("[CameraThread] model allocate error:", e)
                    self.interpreter = None
        else:
            if self.model_path:
                print(f"[CameraThread] Model not found at {self.model_path}; camera-only.")

        self.cap = None

    def _open_camera_try_paths(self, preferred_index=0, out_w=None, out_h=None):
        # prefer env specified device
        pref = PREFERRED_DEVICE
        if pref:
            try:
                cap = cv2.VideoCapture(pref)
                if cap is not None and cap.isOpened():
                    print(f"[CameraThread] opened preferred device {pref} (default backend)")
                    if out_w and out_h:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, out_w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, out_h)
                    return cap
                try: cap.release()
                except: pass
                cap = cv2.VideoCapture(pref, cv2.CAP_V4L2)
                if cap is not None and cap.isOpened():
                    print(f"[CameraThread] opened preferred device {pref} (CAP_V4L2)")
                    if out_w and out_h:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, out_w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, out_h)
                    return cap
                try: cap.release()
                except: pass
            except Exception as e:
                print("[CameraThread] preferred open error:", e)

        # try default/backends...
        try:
            cap = cv2.VideoCapture(preferred_index)
            if cap is not None and cap.isOpened():
                print(f"[CameraThread] opened index {preferred_index} (default backend)")
                if out_w and out_h:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, out_w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, out_h)
                return cap
            try: cap.release()
            except: pass
        except Exception:
            pass

        try:
            cap = cv2.VideoCapture(preferred_index, cv2.CAP_V4L2)
            if cap is not None and cap.isOpened():
                print(f"[CameraThread] opened index {preferred_index} (CAP_V4L2)")
                if out_w and out_h:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, out_w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, out_h)
                return cap
            try: cap.release()
            except: pass
        except Exception:
            pass

        devs = sorted(glob.glob("/dev/video*"))
        if '/dev/video0' in devs:
            devs.remove('/dev/video0'); devs.insert(0, '/dev/video0')
        for dev in devs:
            try:
                cap = cv2.VideoCapture(dev)
                if cap is not None and cap.isOpened():
                    print(f"[CameraThread] opened device {dev} (default backend)")
                    if out_w and out_h:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, out_w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, out_h)
                    return cap
                try: cap.release()
                except: pass
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if cap is not None and cap.isOpened():
                    print(f"[CameraThread] opened device {dev} (CAP_V4L2)")
                    if out_w and out_h:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, out_w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, out_h)
                    return cap
                try: cap.release()
                except: pass
            except Exception as e:
                print(f"[CameraThread] device {dev} open error: {e}")

        # fallback sample.mp4
        sample = os.path.join(os.path.dirname(__file__), "sample.mp4")
        if os.path.exists(sample):
            try:
                cap = cv2.VideoCapture(sample)
                if cap.isOpened():
                    print("[CameraThread] using sample.mp4 fallback")
                    return cap
            except Exception as e:
                print("[CameraThread] sample open error:", e)
        return None

    def run(self):
        self.cap = self._open_camera_try_paths(preferred_index=self.cam_index, out_w=self.out_w, out_h=self.out_h)
        if self.cap is None:
            print("[CameraThread] No camera available; serving placeholder frames.")
            while not self.stop_event.is_set():
                placeholder = np.zeros((self.out_h, self.out_w, 3), dtype=np.uint8)
                cv2.putText(placeholder, "No camera detected", (10, int(self.out_h/2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,200,200), 2, cv2.LINE_AA)
                with self.frame_lock:
                    self.latest_frame = placeholder
                time.sleep(0.2)
            return

        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.out_w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.out_h)
        except Exception:
            pass

        failure_count = 0
        MAX_FAILURES = 6

        while not self.stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                failure_count += 1
                if failure_count >= MAX_FAILURES:
                    print(f"[CameraThread] read failed {failure_count} times; attempting reopen")
                    try:
                        self.cap.release()
                    except:
                        pass
                    time.sleep(0.2)
                    self.cap = self._open_camera_try_paths(preferred_index=self.cam_index, out_w=self.out_w, out_h=self.out_h)
                    failure_count = 0
                    if self.cap is None:
                        time.sleep(0.5)
                        continue
                    else:
                        continue
                else:
                    time.sleep(0.05)
                    continue
            failure_count = 0

            annotated = frame.copy()
            self._frame_counter += 1

            # inference (robust)
            if self.interpreter and (self._frame_counter % self.INFERENCE_EVERY_N == 0):
                try:
                    img = cv2.resize(frame, (self.model_width, self.model_height))
                    input_data = np.expand_dims(img, axis=0)
                    if self.floating_model:
                        input_data = (np.float32(input_data) - self.input_mean) / self.input_std
                    else:
                        input_data = np.uint8(input_data)
                    self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
                    t0 = time.time()
                    self.interpreter.invoke()
                    inference_time = (time.time() - t0) * 1000.0

                    output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
                    if self.output_details[0]['dtype'] == np.uint8:
                        q = self.output_details[0]['quantization']
                        if q:
                            scale, zero_point = q
                            if scale:
                                output_data = scale * (output_data - zero_point)

                    top_idx = int(np.argmax(output_data))
                    prob = float(output_data[top_idx])
                    label_name = self.labels[top_idx] if (0 <= top_idx < len(self.labels)) else str(top_idx)
                    self._last_label = (label_name, prob, int(inference_time))
                    label_text = f"{label_name}: {prob:.2f} ({int(inference_time)}ms)"
                    cv2.putText(annotated, label_text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
                    cv2.rectangle(annotated, (10,36), (10 + int(prob*250), 54), (0,255,0), -1)
                except Exception as e:
                    print("[CameraThread] inference error:", e)
                    if self._last_label:
                        ln, pr, mt = self._last_label
                        label_text = f"{ln}: {pr:.2f} ({mt}ms)"
                        cv2.putText(annotated, label_text, (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,0), 2, cv2.LINE_AA)
                        cv2.rectangle(annotated, (10,36), (10 + int(max(0.0, min(1.0, pr))*250), 54), (200,200,0), -1)
                    else:
                        cv2.putText(annotated, time.strftime("%Y-%m-%d %H:%M:%S"), (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255),1,cv2.LINE_AA)
            else:
                if self._last_label:
                    ln, pr, mt = self._last_label
                    label_text = f"{ln}: {pr:.2f} ({mt}ms)"
                    cv2.putText(annotated, label_text, (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180,180,180), 2, cv2.LINE_AA)
                    cv2.rectangle(annotated, (10,36), (10 + int(max(0.0, min(1.0, pr))*250),54), (180,180,180), -1)
                else:
                    cv2.putText(annotated, time.strftime("%Y-%m-%d %H:%M:%S"), (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255),1,cv2.LINE_AA)

            if annotated.shape[1] != self.out_w or annotated.shape[0] != self.out_h:
                annotated = cv2.resize(annotated, (self.out_w, self.out_h))

            # store latest frame
            with self.frame_lock:
                self.latest_frame = annotated.copy()

            # On inference frames, publish telemetry (label + prob)
            if self._last_label and (self._frame_counter % self.INFERENCE_EVERY_N == 0):
                try:
                    ln, pr, mt = self._last_label
                    tele = {"label": ln, "prob": round(float(pr),3), "ms": mt, "ts": time.time()}
                    if mqtt_client:
                        mqtt_client.publish(TOPIC_TELE, json.dumps(tele))
                    # optionally publish small JPG thumbnail
                    if PUBLISH_THUMB:
                        try:
                            small = cv2.resize(annotated, (160, 90))
                            ret, jpg = cv2.imencode('.jpg', small, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
                            if ret:
                                b64 = base64.b64encode(jpg.tobytes()).decode('ascii')
                                mqtt_client.publish(TOPIC_THUMB, b64)
                        except Exception as e:
                            print("[CameraThread] thumb publish error:", e)
                except Exception as e:
                    print("[CameraThread] telemetry publish error:", e)

        try:
            self.cap.release()
        except:
            pass
        print("[CameraThread] stopped")

    def get_frame_rgb(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            bgr = self.latest_frame.copy()
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def stop(self):
        self.stop_event.set()

# --- MJPEG encoder (unchanged) ---
_latest_jpeg = None
_latest_lock = threading.Lock()
_stop_enc = threading.Event()
_enc_thread = None

def encoder_loop(cam_thread_obj):
    global _latest_jpeg
    while not _stop_enc.is_set():
        rgb = cam_thread_obj.get_frame_rgb()
        if rgb is None:
            time.sleep(0.02)
            continue
        try:
            small = cv2.resize(rgb, (STREAM_W, STREAM_H), interpolation=cv2.INTER_AREA)
            bgr = cv2.cvtColor(small, cv2.COLOR_RGB2BGR)
            ret, jpg = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ret:
                time.sleep(0.01)
                continue
            with _latest_lock:
                _latest_jpeg = jpg.tobytes()
        except Exception as e:
            print("[Encoder] error:", e)
        time.sleep(ENCODE_INTERVAL)

def mjpeg_gen():
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    while True:
        with _latest_lock:
            data = _latest_jpeg
        if data is None:
            time.sleep(0.02)
            continue
        yield boundary + data + b"\r\n"

# --- Auth helper (unchanged) ---
def check_token_from_request(request: Request) -> bool:
    token = None
    auth = request.headers.get("authorization")
    if auth and auth.startswith("Bearer "):
        token = auth.split(" ",1)[1].strip()
    if not token:
        token = request.query_params.get("token")
    if AUTH_TOKEN:
        if not token or token != AUTH_TOKEN:
            raise HTTPException(status_code=401, detail="Missing/invalid token")
    return True

# --- MQTT client: subscribes to TOPIC_CMD, calls set_motor -- lightweight mapping ---
mqtt_client = None
def on_mqtt_connect(client, userdata, flags, rc):
    print(f"[MQTT] connected rc={rc}")
    client.subscribe(TOPIC_CMD)
def on_mqtt_message(client, userdata, msg):
    try:
        payload = msg.payload.decode('utf-8')
        data = json.loads(payload)
        # If payload contains discrete command:
        if isinstance(data, dict) and data.get("_cmd"):
            cmd = data["_cmd"].lower()
            speed = (data.get("speed") or DEFAULT_SPEED) / 255.0
            if cmd == "forward": set_motor('left', speed); set_motor('right', speed)
            elif cmd == "back": set_motor('left', -speed); set_motor('right', -speed)
            elif cmd == "left": set_motor('left', -speed); set_motor('right', speed)
            elif cmd == "right": set_motor('left', speed); set_motor('right', -speed)
            elif cmd == "up": set_motor('top', speed)
            elif cmd == "down": set_motor('bottom', speed)
            elif cmd == "stop": stop_all()
            return
        # If numeric mapping provided
        if isinstance(data, dict):
            for k in ("left","right","top","bottom"):
                if k in data:
                    try:
                        set_motor(k, float(data[k]))
                    except Exception as e:
                        print("[MQTT] set_motor error:", e)
    except Exception as e:
        print("[MQTT] message error:", e)

def mqtt_start():
    global mqtt_client
    if not MQTT_BROKER or mqtt is None:
        print("[MQTT] broker not configured or paho-mqtt missing; remote control disabled")
        return
    client = mqtt.Client()
    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASS or None)
    client.on_connect = on_mqtt_connect
    client.on_message = on_mqtt_message
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        client.loop_start()
        mqtt_client = client
        print("[MQTT] started and connected")
    except Exception as e:
        print("[MQTT] connect error:", e)

# --- Endpoints (unchanged behavior) ---
cam_thread: Optional[CameraThread] = None

@app.get("/")
def root():
    return {"status":"online","video":"/video","controls":"/controls.html"}

@app.get("/controls.html")
def controls_html():
    path = os.path.join(os.path.dirname(__file__), "controls.html")
    if os.path.exists(path):
        return FileResponse(path, media_type="text/html")
    raise HTTPException(status_code=404, detail="controls.html not found")

@app.get("/video")
def video():
    return StreamingResponse(mjpeg_gen(), media_type='multipart/x-mixed-replace; boundary=frame')

@app.get("/control", dependencies=[Depends(check_token_from_request)])
def control(dir: str = Query(...)):
    speed = DEFAULT_SPEED/255.0
    d = dir.lower()
    if d == "forward":
        set_motor('left', speed); set_motor('right', speed)
    elif d == "back":
        set_motor('left', -speed); set_motor('right', -speed)
    elif d == "left":
        set_motor('left', -speed); set_motor('right', speed)
    elif d == "right":
        set_motor('left', speed); set_motor('right', -speed)
    elif d == "up":
        set_motor('top', speed)
    elif d == "down":
        set_motor('bottom', speed)
    else:
        return {"error":"invalid direction"}
    return {"moved": d}

@app.get("/stop", dependencies=[Depends(check_token_from_request)])
def stop():
    stop_all()
    return {"status":"stopped"}

# Local WebSocket endpoint (keeps previous functionality for local mobile browser)
@app.websocket("/ws")
async def ws_control(ws: WebSocket):
    await ws.accept()
    try:
        qs_bytes = ws.scope.get("query_string", b"")
        token_q = None
        if qs_bytes:
            qs = qs_bytes.decode()
            for kv in qs.split("&"):
                if "=" in kv:
                    k,v = kv.split("=",1)
                    if k == "token":
                        token_q = v; break

        if AUTH_TOKEN:
            if token_q:
                if token_q != AUTH_TOKEN:
                    await ws.close(code=4003); return
                await ws.send_text(json.dumps({"status":"auth_ok"}))
            else:
                try:
                    txt = await ws.receive_text()
                    obj = json.loads(txt)
                    if not isinstance(obj, dict) or obj.get("token") != AUTH_TOKEN:
                        await ws.close(code=4003); return
                    await ws.send_text(json.dumps({"status":"auth_ok"}))
                except Exception:
                    await ws.close(code=4002); return
        else:
            await ws.send_text(json.dumps({"status":"no_auth"}))

        while True:
            txt = await ws.receive_text()
            try:
                data = json.loads(txt)
            except Exception:
                continue
            # support both numeric motor map and discrete _cmd
            if isinstance(data, dict) and data.get("_cmd"):
                # keep previous behavior
                cmd = data["_cmd"].lower()
                speed = (data.get("speed") or DEFAULT_SPEED) / 255.0
                if cmd == "forward": set_motor('left', speed); set_motor('right', speed)
                elif cmd == "back": set_motor('left', -speed); set_motor('right', -speed)
                elif cmd == "left": set_motor('left', -speed); set_motor('right', speed)
                elif cmd == "right": set_motor('left', speed); set_motor('right', -speed)
                elif cmd == "up": set_motor('top', speed)
                elif cmd == "down": set_motor('bottom', speed)
                elif cmd == "stop": stop_all()
            else:
                for k in ("left","right","top","bottom"):
                    if k in data:
                        try:
                            set_motor(k, float(data[k]))
                        except Exception as e:
                            print("[ws] set_motor error:", e)
            await ws.send_text(json.dumps({"t": time.time()}))
    except WebSocketDisconnect:
        stop_all()
    except Exception as e:
        print("[ws] exception:", e)
        stop_all()
        try:
            await ws.close()
        except:
            pass

# --- Startup / shutdown ---
@app.on_event("startup")
def on_startup():
    global cam_thread, _enc_thread, _stop_enc
    print("[Server] startup: creating camera thread and encoder...")
    MODEL_PATH = "/home/asd/my_model.tflite"
    LABELS_PATH = "/home/asd/labels.txt"
    cam_thread = CameraThread(cam_index=0, model_path=MODEL_PATH, labels_path=LABELS_PATH, out_w=STREAM_W, out_h=STREAM_H)
    cam_thread.start()
    _stop_enc.clear()
    enc = threading.Thread(target=encoder_loop, args=(cam_thread,), daemon=True)
    enc.start()
    global _enc_thread
    _enc_thread = enc

    # start mqtt client in background if configured
    threading.Thread(target=mqtt_start, daemon=True).start()

    print("[Server] startup complete.")

@app.on_event("shutdown")
def on_shutdown():
    global cam_thread, _enc_thread, _stop_enc, mqtt_client
    print("[Server] shutdown requested.")
    try:
        _stop_enc.set()
    except:
        pass
    try:
        if cam_thread:
            cam_thread.stop()
    except:
        pass
    try:
        if _enc_thread:
            _enc_thread.join(timeout=1.0)
    except:
        pass
    try:
        stop_all()
    except:
        pass
    try:
        if mqtt_client:
            mqtt_client.loop_stop(); mqtt_client.disconnect()
    except:
        pass
    print("[Server] shutdown complete.")
