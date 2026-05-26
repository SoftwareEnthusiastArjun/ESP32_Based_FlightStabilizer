# GCSHelper — ESP32 Flight Stabilizer Configurator

**Part of the ESP32 Flight Stabilization System**
*M.Sc. Thesis — Arjun Veeramony, FH Dortmund*

---

GCSHelper is the setup and configuration utility for the ESP32-based
Flight Stabilizer. Use it before flying — to flash firmware and
configure WiFi credentials over USB.

## What it does

- **WiFi Setup** — configure the ESP32 WiFi credentials over USB serial
  at first boot, no hardcoding needed
- **Flash Firmware** — flash pre-compiled `.bin` firmware files directly
  to the ESP32 using esptool, no PlatformIO or Arduino IDE required
- **Serial Monitor** — view live serial output from the ESP32 for
  debugging and verification

## Requirements

- Windows 10 / 11
- ESP32 connected via USB
- `pip install pyserial esptool`

## Run from source

```bash
pip install -r requirements.txt
python flight_serial_monitor_gui.py
```

## Files

```
GCSHelper/
├── flight_serial_monitor_gui.py   ← main app (run this)
├── flash_tab.py                   ← firmware flash module
├── requirements.txt
├── LICENSE.txt
└── README.md
```

## Related App

**FlightGCS** — the wireless Ground Control Station for real-time
PID tuning, RC monitoring and orientation HUD over WiFi.

## Citation

```
Arjun Veeramony, "ESP32-Based Flight Stabilization System with
Wireless Ground Control Station," M.Sc. Thesis, FH Dortmund, 2025.
```

## License

MIT-based open source — see LICENSE.txt
