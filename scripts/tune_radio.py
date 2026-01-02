import time
import json
import sys
import paho.mqtt.client as mqtt

# --- CONFIGURATION ---
MQTT_BROKER = "192.168.1.100"  # <--- CHANGE TO YOUR MQTT IP
MQTT_PORT = 1883
MQTT_USER = "homeassistant"  # <--- CHANGE OR KEEP NONE
MQTT_PASS = "password"  # <--- CHANGE OR KEEP NONE

# Your Device ID (e.g. "18:005960")
DEVICE_ID = "18:005960"  # <--- CHANGE THIS TO MATCH YOUR DEVICE

# Topics
TOPIC_ROOT = f"RAMSES/GATEWAY/{DEVICE_ID}"
TOPIC_CMD = f"{TOPIC_ROOT}/cmd/cmd"
TOPIC_CMD_RESULT = f"{TOPIC_ROOT}/cmd/result"
TOPIC_RX = f"{TOPIC_ROOT}/rx"

# State Tracking
tuning_active = False
start_time = 0
last_freq = "Unknown"


def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT Broker (Result Code: {rc})")
    client.subscribe(TOPIC_CMD_RESULT)
    client.subscribe(TOPIC_RX)
    print("Subscribed to monitor topics.")

    # Start the tuning process
    print("\n--- STARTING TUNER ---")
    print(f"Sending '!FT' to {TOPIC_CMD}...")
    client.publish(TOPIC_CMD, "!FT")

    global start_time, tuning_active
    start_time = time.time()
    tuning_active = True


def on_message(client, userdata, msg):
    global tuning_active, last_freq

    try:
        payload = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        return  # Ignore non-JSON

    # 1. Monitor Frequency Changes (RX Topic)
    if msg.topic == TOPIC_RX:
        # The tuner outputs pseudo-packets looking like "!c <reg> <val>..."
        message_str = payload.get("msg", "")
        if message_str.startswith("!c"):
            # Format: !c 21 65 6A 6B
            parts = message_str.split()
            if len(parts) >= 4:
                freq_hex = "".join(parts[2:5])
                last_freq = freq_hex
                print(
                    f"[TUNER UPDATE] Frequency Adjusted: {message_str} (Hex: {freq_hex})"
                )

    # 2. Monitor Status Checks (CMD Result Topic)
    elif msg.topic == TOPIC_CMD_RESULT:
        # Response format: {"cmd": "!F", "return": "# !F F=21656A tuning"}
        cmd_sent = payload.get("cmd", "")
        result_str = payload.get("return", "")

        if cmd_sent == "!F":
            print(f"[STATUS] {result_str.strip()}")

            # Check if "tuning" is present in the response
            if "tuning" not in result_str:
                print("\n--- TUNING COMPLETE ---")
                print(f"Final Frequency: {last_freq}")

                # Save the result
                print("Saving settings to NVS (!FS)...")
                client.publish(TOPIC_CMD, "!FS")
                time.sleep(1)

                print("Done. You may press Ctrl+C to exit.")
                tuning_active = False
                client.disconnect()
                sys.exit(0)
            else:
                print("... Tuning in progress ...")


client = mqtt.Client()
if MQTT_USER:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

client.on_connect = on_connect
client.on_message = on_message

try:
    print(f"Connecting to {MQTT_BROKER}...")
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    # Start the loop in a non-blocking way so we can run our own timer
    client.loop_start()

    # Main "Watchdog" Loop
    while True:
        if tuning_active:
            # Every 10 seconds, ask the device for its status
            client.publish(TOPIC_CMD, "!F")

        time.sleep(10)

except KeyboardInterrupt:
    print("\nExiting...")
    client.disconnect()
    client.loop_stop()
