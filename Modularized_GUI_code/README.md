# FlightGCS — Ground Control Station

**Part of the ESP32 Flight Stabilization System**
*M.Sc. Thesis — Arjun Veeramony, FH Dortmund*

---

FlightGCS is a wireless Ground Control Station for the ESP32-based flight stabilizer.
Connects over WiFi — tune PIDs, monitor RC inputs, view live HUD, configure in real time.

## Download
👉 [FlightGCS_Setup_v1.0.0.exe](https://github.com/SoftwareEnthusiastArjun/ESP32_Based_FlightStabilizer)

## Run from source
```bash
pip install -r requirements.txt
python main.py
```

## Structure
```
FlightGCS/
├── main.py               ← python main.py to run
├── requirements.txt
├── LICENSE.txt
├── icon.ico
├── build.spec            ← PyInstaller
├── installer.iss         ← Inno Setup
├── gcs/
│   ├── theme.py
│   ├── connection.py
│   ├── hud.py
│   ├── control_tab.py
│   ├── servo_tab.py
│   ├── visualizer_tab.py
│   ├── pid_tab.py
│   └── diagnostics_tab.py
├── firmware/firmware_final.cpp
└── tools/diagnose_network.py
```

## Citation
Arjun Veeramony, "ESP32-Based Flight Stabilization System with Wireless Ground Control Station,"
M.Sc. Thesis, FH Dortmund, 2025.

## License
MIT-based open source — see LICENSE.txt
