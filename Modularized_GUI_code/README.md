# ESP32 Flight Stabilizer — Project

## Folder Structure

```
FlightGCS/
├── main.py                        ← Launch GCS here:  python main.py
├── requirements.txt
│
├── gcs/                           ← GCS Python package
│   ├── __init__.py
│   ├── theme.py                   ← Shared dark theme + helpers
│   ├── connection.py              ← TCP connection + auto-discovery
│   ├── hud.py                     ← Flight HUD renderer (pygame)
│   ├── control_tab.py             ← ⚙  Filters tab
│   ├── servo_tab.py               ← 🎮  RC Input tab
│   ├── visualizer_tab.py          ← 🛩  HUD tab
│   ├── pid_tab.py                 ← ↕↔  Pitch / Roll PID tabs
│   └── diagnostics_tab.py        ← 📊  Diagnostics tab
│
├── serial_monitor/
│   └── flight_serial_monitor_gui.py  ← Serial monitor + WiFi config GUI
│                                        Run:  python flight_serial_monitor_gui.py
│
├── firmware/
│   └── firmware_final.cpp         ← ESP32 Arduino firmware
│
└── tools/
    └── diagnose_network.py        ← Network diagnostic script
```

## Quick Start

### 1. Install dependencies
```
pip install -r requirements.txt
```

### 2. Flash the firmware
Open `firmware/firmware_final.cpp` in Arduino IDE, select your ESP32 board, and upload.

### 3. Configure WiFi on the ESP32
```
cd serial_monitor
python flight_serial_monitor_gui.py
```
- Connect to the ESP32 serial port
- Reset the ESP32 — the boot menu appears automatically
- Click **2 Change WiFi** → enter credentials → press **Send**

### 4. Launch the GCS
```
cd ..          # back to FlightGCS/
python main.py
```
- Click **⚡ Auto Connect** — the GCS scans the network and connects automatically

## GCS Tabs

| Tab | Purpose |
|-----|---------|
| ⚙  Filters | ACCEL / GYRO / COMP filter coefficients |
| 🎮  RC Input | Live bidirectional stick position bars |
| 🛩  HUD | Artificial horizon, pitch ladder, compass |
| ↕  Pitch PID | Kp / Ki / Kd for pitch axis |
| ↔  Roll PID | Kp / Ki / Kd for roll axis |
| 📊  Diagnostics | Live angles, axis config, calibration, deadbands |

## Network Troubleshooting
If Auto Connect fails, run the diagnostic tool:
```
python tools/diagnose_network.py
```
Common fixes:
- Disable **AP Isolation** in your router settings
- Make sure laptop and ESP32 are on the **same WiFi network**
- Use **Manual IP** in the GCS if Auto Connect can't find the ESP32
