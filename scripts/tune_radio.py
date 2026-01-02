import time
import json
import sys
import re
import paho.mqtt.client as mqtt
from datetime import datetime

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

# State Tracking
tuning_active = False
saving_active = False
start_time = 0
last_freq = "Unknown"


# --- Helper Function for Logging ---
def log(msg):
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] {msg}")


# --- Callbacks ---


def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        log(f"Connected to MQTT Broker (Result Code: {reason_code})")
        client.subscribe(TOPIC_CMD_RESULT)
        log("Subscribed to monitor topics.")

        # Start the tuning process
        log("\n--- STARTING TUNER ---")
        log(f"Sending '!FT' to {TOPIC_CMD}...")
        client.publish(TOPIC_CMD, "!FT")

        global start_time, tuning_active
        start_time = time.time()
        tuning_active = True
    else:
        log(f"Failed to connect, return code {reason_code}")


def on_message(client, userdata, msg):
    global tuning_active, saving_active, last_freq

    try:
        payload = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        return

    if msg.topic == TOPIC_CMD_RESULT:
        cmd_sent = payload.get("cmd", "")
        result_str = payload.get("return", "")

        # 1. Handle Status Updates (!F)
        if cmd_sent == "!F":
            # Print the raw status from the device
            log(f"[STATUS] {result_str.strip()}")

            # Extract frequency from string like "!F F=21656a tuning"
            match = re.search(r"F=([0-9a-fA-F]{6})", result_str)
            if match:
                last_freq = match.group(1)

            # Check if tuning is still happening
            if "tuning" not in result_str and tuning_active:
                log("\n--- TUNING COMPLETE ---")
                log(f"Final Frequency Detected: {last_freq}")

                # Stop the polling loop
                tuning_active = False

                # Initiate Save
                log("Sending Save Command (!FS)...")
                client.publish(TOPIC_CMD, "!FS")
                saving_active = True

            elif tuning_active:
                log(f"... Tuning in progress (Current: {last_freq}) ...")

        # 2. Handle Save Confirmation (!FS)
        elif cmd_sent == "!FS" and saving_active:
            if "Saved" in result_str:
                log("\n[SUCCESS] Settings verified as SAVED to NVS.")
                log(f"Stored Frequency: {last_freq}")
            else:
                log(
                    f"\n[WARNING] received response to save, but 'Saved' text missing: {result_str}"
                )

            log("Exiting...")
            client.disconnect()
            sys.exit(0)


# --- Main Execution ---

# Use CallbackAPIVersion.VERSION2 to silence deprecation warning
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

if MQTT_USER:
    client.username_pw_set(MQTT_USER, MQTT_PASS)

client.on_connect = on_connect
client.on_message = on_message

try:
    log(f"Connecting to {MQTT_BROKER}...")
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    # Start the loop
    client.loop_start()

    while True:
        if tuning_active:
            # Poll status every 10 seconds
            client.publish(TOPIC_CMD, "!F")
            time.sleep(10)
        elif saving_active:
            # We are waiting for the save confirmation
            time.sleep(0.5)
        else:
            # Just starting up
            time.sleep(0.5)

except KeyboardInterrupt:
    log("\nExiting...")
    client.disconnect()
    client.loop_stop()
