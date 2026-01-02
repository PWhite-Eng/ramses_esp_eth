# CC1101 Frequency Tuning Tool

The `scripts` directory contains `tune_radio.py`, a Python automation script designed to optimize the reception quality of the `ramses_esp_eth` gateway.

## What is this?
The CC1101 radio module and the Honeywell Ramses devices it communicates with rely on quartz crystals to generate their radio frequency (868.3 MHz). Due to manufacturing tolerances, temperature, and aging, no two crystals are exactly alike.

If your gateway is listening at **868.300 MHz** but your thermostat is transmitting at **868.305 MHz**, the mismatch causes **Pulse Width Distortion**.
* The "0" and "1" pulses become distorted (one gets fatter, one gets thinner).
* Short packets might survive, but long packets often fail at the very end with CRC or Manchester errors (e.g., `* ERR:04`).

**This tool automatically scans frequencies to find the "center" point where the signal is strongest and cleanest.**

## How it works
The script acts as an external "manager" for the gateway's built-in tuning state machine:
1.  **Connects** to your MQTT broker.
2.  **Triggers** the tuning mode by sending the `!FT` command.
3.  **Monitors** the gateway's output for frequency adjustments (messages starting with `!c`).
4.  **Polls** the status every 10 seconds using `!F`.
5.  **Detects Completion:** When the gateway stops reporting "tuning" status, the script detects stability.
6.  **Saves:** It automatically sends the `!FS` command to save the new frequency to the ESP32's Non-Volatile Storage (NVS).

## Prerequisites
You need Python installed on your computer. You also need the MQTT library:

```bash
pip install paho-mqtt
```
## How to use it
1. Configure the script
Open tune_radio.py in a text editor and update the Configuration Section at the top:

```Python
# --- CONFIGURATION ---
MQTT_BROKER = "192.168.1.100"   # Your Home Assistant / MQTT Broker IP
MQTT_USER   = "homeassistant"   # Your MQTT Username (or None)
MQTT_PASS   = "password"        # Your MQTT Password (or None)
DEVICE_ID   = "18:005960"       # The ID of your ramses_esp_eth device
```

2. Run the script
Open your terminal in this folder and run:
```Python
python tune_radio.py
```

3. Wait
The process takes **5 to 15 minutes**.

- You will see output like `[TUNER UPDATE] Frequency Adjusted...` as it hones in on the signal.
- Do not stop the script until it says `--- TUNING COMPLETE ---`.
- The script will automatically save the settings and exit when finished.

## Frequently Asked Questions

### Do I need to run this all the time?

**No**. You typically run this **once** when you install the device. Frequency drift happens over years (aging) or extreme temperature shifts. Once tuned, the setting remains valid for a very long time.

### Should I automate this in Home Assistant?

**No**. Running the tuner involves actively changing the receive frequency, which causes the gateway to **miss packets** while scanning. It should only be run manually or as a "repair" action if the device stops receiving data entirely.

### Does this tune the Baud Rate?

**No**. This tool tunes the **Carrier Frequency (MHz)**.

- **Carrier Frequency:** Matches the radio waves between devices. Fixed by this tool.
- **Baud Rate:** Matches the timing of bits (38.4 kbps). This is fixed in firmware (`RADIO_BAUD_RATE = 38383`) and does not need tuning.

### Will I lose the setting if I update the firmware?

**No**. The frequency setting is saved to a special area of memory (NVS) that survives firmware updates. You only lose it if you perform a "Full Chip Erase".