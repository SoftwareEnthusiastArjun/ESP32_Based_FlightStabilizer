#!/usr/bin/env python3
"""
ESP32 GCS Network Diagnostic
Run this on the same PC as the GCS to diagnose connection issues.
"""
import socket
import subprocess
import sys
import os

ESP32_IP = "192.168.0.112"
ESP32_PORT = 12345

print("=" * 55)
print("  ESP32 GCS Network Diagnostic")
print("=" * 55)

# 1. Get laptop IP
print("\n[1] Laptop network interfaces:")
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    laptop_ip = s.getsockname()[0]
    s.close()
    print(f"    Active IP : {laptop_ip}")
    
    esp_subnet  = ".".join(ESP32_IP.split(".")[:3])
    my_subnet   = ".".join(laptop_ip.split(".")[:3])
    if esp_subnet == my_subnet:
        print(f"    Subnet    : SAME as ESP32 ({esp_subnet}.x) ✔")
    else:
        print(f"    Subnet    : DIFFERENT from ESP32!")
        print(f"    ESP32 is on {esp_subnet}.x, laptop is on {my_subnet}.x")
        print(f"    → Connect laptop to the SAME WiFi network as ESP32")
except Exception as e:
    print(f"    Error: {e}")

# 2. Ping test
print(f"\n[2] Ping {ESP32_IP}:")
try:
    result = subprocess.run(
        ["ping", "-n", "2", "-w", "1000", ESP32_IP],
        capture_output=True, text=True, timeout=10
    )
    lines = [l for l in result.stdout.splitlines() if l.strip()]
    for l in lines[-4:]:
        print(f"    {l}")
    if "TTL=" in result.stdout or "bytes=" in result.stdout:
        print("    → ESP32 is reachable at network level ✔")
    else:
        print("    → ESP32 is NOT pingable")
        print("    → Check: same WiFi network? AP isolation?")
except Exception as e:
    print(f"    Error: {e}")

# 3. TCP port test
print(f"\n[3] TCP connect to {ESP32_IP}:{ESP32_PORT}:")
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3)
    result = s.connect_ex((ESP32_IP, ESP32_PORT))
    s.close()
    if result == 0:
        print(f"    Port {ESP32_PORT} is OPEN ✔  — GCS should connect fine")
    elif result == 111 or result == 10061:
        print(f"    Port {ESP32_PORT} REFUSED — ESP32 is reachable but TCP server not running")
        print(f"    → Check: is the ESP32 WiFi LED on (blue)?")
        print(f"    → Check serial monitor — did WiFi connect successfully?")
    else:
        print(f"    Port {ESP32_PORT} TIMEOUT (code {result})")
        print(f"    → Router has AP isolation / client isolation enabled")
        print(f"    → Or laptop is on a different subnet/band (2.4GHz vs 5GHz)")
except Exception as e:
    print(f"    Error: {e}")

# 4. Windows Firewall hint
print(f"\n[4] Windows Firewall:")
print(f"    If ping works but TCP fails, Windows Firewall may be blocking")
print(f"    outbound connections to port {ESP32_PORT}.")
print(f"    → Try: temporarily disable Windows Firewall and retry")
print(f"    → Or: add outbound rule for port {ESP32_PORT}")

print("\n" + "=" * 55)
print("  Done. Share this output to diagnose the issue.")
print("=" * 55)
