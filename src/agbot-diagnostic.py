#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import shutil

# --- Terminal Styling ---
BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

def header(text):
    print(f"\n{BOLD}{BLUE}=== {text} ==={RESET}")

def run_check(label, cmd):
    try:
        result = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT).decode().strip()
        print(f"{GREEN}[✓] {label}:{RESET} {result}")
        return result
    except:
        print(f"{RED}[✗] {label}: FAILED or NOT FOUND{RESET}")
        return None

header("SYSTEM & HARDWARE PROBE")

# 1. Check Serial Port Hardware Existence
run_check("ZED-F9P GPS (ACM0)", "ls -l /dev/ttyACM0")
run_check("Controller (ACM1)", "ls -l /dev/ttyACM1")

# 2. Probe Serial Data Throughput (Verifying 115200 Baud)
print(f"{YELLOW}[?] Probing ACM0 Data Stream (3s)...{RESET}")
# This reads the serial port for 3 seconds and counts bytes
cmd_probe = "timeout 3 cat /dev/ttyACM0 | wc -c"
bytes_count = run_check("ACM0 Byte Count", cmd_probe)
if bytes_count and int(bytes_count) > 1000:
    print(f"    {GREEN}High-speed data detected. Baudrate is likely correct.{RESET}")
else:
    print(f"    {RED}Low or No data. Check physical connection or Baudrate!{RESET}")

# 3. Network & WebUI Port
run_check("WebUI Port 8080", "netstat -tuln | grep 8080")

header("ROS 2 ENVIRONMENT PROBE")

# 1. RMW Check
run_check("Middleware (RMW)", "printenv RMW_IMPLEMENTATION")

# 2. Node Existence
nodes = run_check("Active ROS Nodes", "ros2 node list")
if nodes:
    for n in ["ublox_gps_node", "basekit_driver_node", "basekit_ui_node"]:
        if n in nodes:
            print(f"    {GREEN}[✓] {n} is running.{RESET}")
        else:
            print(f"    {RED}[!] {n} is MISSING!{RESET}")

# 3. Topic Frequency Probe (The ultimate test)
header("TOPIC HEALTH (LIVE PROBE)")
topics = [
    ("/ublox_gps_node/fix", "GPS Fix"),
    ("/motor_status", "Motor Data"),
]

for topic, label in topics:
    print(f"{YELLOW}[?] Testing {label} frequency...{RESET}")
    # We try to get the hz of the topic for 2 seconds
    cmd_hz = f"timeout 2 ros2 topic hz {topic}"
    try:
        output = subprocess.check_output(cmd_hz, shell=True).decode().strip()
        if "average rate" in output:
            rate = output.split("average rate:")[1].split("\n")[0].strip()
            print(f"    {GREEN}[✓] {label} is ALIVE at {rate} Hz{RESET}")
        else:
            print(f"    {RED}[✗] {label} has NO DATA flow.{RESET}")
    except:
        print(f"    {RED}[✗] {label} is NOT PUBLISHING.{RESET}")

header("DIAGNOSTIC COMPLETE")
