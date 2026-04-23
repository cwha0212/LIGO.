import json
import time
import paho.mqtt.client as mqtt

topic = "navi1/control/mode"
payload = {"command": "start", "mapping_mode": False, "map_name": "site_a_20260422"}
# payload = {"command": "stop"}
state = {"connected": False, "published": False, "failed": False, "mid": None}


def _reason_failed(reason_code: object) -> bool:
    if reason_code is None:
        return False
    if hasattr(reason_code, "is_failure") and callable(reason_code.is_failure):
        try:
            return bool(reason_code.is_failure)
        except Exception:
            pass
    if isinstance(reason_code, int):
        return reason_code != 0
    try:
        return int(reason_code) != 0
    except (TypeError, ValueError):
        pass
    return str(reason_code).strip().lower() not in ("success", "0", "no error", "no_error")


def on_connect(client, userdata, flags, reason_code, properties):
    if _reason_failed(reason_code):
        print(f"[ERROR] MQTT connect failed: {reason_code!r}")
        state["failed"] = True
        return
    state["connected"] = True
    info = client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1, retain=False)
    state["mid"] = info.mid


def on_publish(client, userdata, mid, reason_code, properties):
    if _reason_failed(reason_code):
        print(f"[ERROR] MQTT publish failed: {reason_code!r}")
        state["failed"] = True
        return
    if state["mid"] is None or mid == state["mid"]:
        state["published"] = True


c = mqtt.Client(transport="websockets", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
c.ws_set_options(path="/mqtt")
c.on_connect = on_connect
c.on_publish = on_publish
c.connect("rms.bottle-tak.com", 80, keepalive=10)
c.loop_start()

t0 = time.time()
while time.time() - t0 < 8.0:
    if state["published"] or state["failed"]:
        break
    time.sleep(0.05)

c.loop_stop()
c.disconnect()

if state["published"]:
    print("[OK] published:", payload)
elif state["connected"]:
    print("[ERROR] connected but publish not confirmed")
else:
    print("[ERROR] connect failed or timed out")