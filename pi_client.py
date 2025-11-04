# pi_client.py
import os, json, time, logging
import paho.mqtt.client as mqtt
from drone_motors import set_motor, stop_all, shutdown   # use your existing module

LOG = logging.getLogger("pi_client")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

MQTT_BROKER = os.getenv("MQTT_BROKER", "broker.hivemq.com")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "drone/controls/mydevice")
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "")
DRONE_AUTH_TOKEN = os.getenv("DRONE_AUTH_TOKEN", "")

client = mqtt.Client(client_id=f"pi-client-{int(time.time())}")

if MQTT_USERNAME and MQTT_PASSWORD:
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

def on_connect(c, userdata, flags, rc):
    LOG.info("Connected to MQTT broker rc=%s", rc)
    c.subscribe(MQTT_TOPIC)
    LOG.info("Subscribed to %s", MQTT_TOPIC)

def process_message(payload):
    # payload is bytes -> JSON
    try:
        obj = json.loads(payload)
    except Exception as e:
        LOG.warning("invalid JSON payload: %s", e)
        return
    # token check
    if DRONE_AUTH_TOKEN:
        if "token" not in obj or obj.get("token") != DRONE_AUTH_TOKEN:
            LOG.warning("message missing/invalid token - ignored")
            return
    # if command style _cmd
    if "_cmd" in obj:
        cmd = obj["_cmd"]
        speed = obj.get("left", None)  # UI often sends left/right floats for discrete commands
        LOG.info("cmd=%s speed=%s", cmd, speed)
        if cmd == "forward":
            set_motor("left", float(obj.get("left", 0.6))); set_motor("right", float(obj.get("right", 0.6)))
        elif cmd == "back":
            set_motor("left", float(obj.get("left", -0.6))); set_motor("right", float(obj.get("right", -0.6)))
        elif cmd == "left":
            set_motor("left", float(obj.get("left", -0.6))); set_motor("right", float(obj.get("right", 0.6)))
        elif cmd == "right":
            set_motor("left", float(obj.get("left", 0.6))); set_motor("right", float(obj.get("right", -0.6)))
        elif cmd == "up":
            set_motor("top", float(obj.get("top", 0.6)))
        elif cmd == "down":
            set_motor("bottom", float(obj.get("bottom", 0.6)))
        elif cmd == "stop":
            stop_all()
        else:
            LOG.warning("unknown discrete command %s", cmd)
        return

    # numeric motor control: direct keys left/right/top/bottom expected -1..1
    for k in ("left","right","top","bottom"):
        if k in obj:
            try:
                set_motor(k, float(obj[k]))
            except Exception as e:
                LOG.warning("set_motor error %s: %s", k, e)

client.on_connect = on_connect

def on_message(c, userdata, msg):
    LOG.info("MQTT msg on %s", msg.topic)
    process_message(msg.payload.decode("utf-8"))

client.on_message = on_message

def main():
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    except Exception as e:
        LOG.error("connect error: %s", e)
        return
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        LOG.info("shutting down")
        stop_all()
        shutdown()

if __name__ == "__main__":
    main()
