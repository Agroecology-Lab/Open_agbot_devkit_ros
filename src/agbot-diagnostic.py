#!/usr/bin/env python3
import os
import subprocess
import time

BLUE, GREEN, YELLOW, RED, BOLD, RESET = "\033[94m", "\033[92m", "\033[93m", "\033[91m", "\033[1m", "\033[0m"

def header(text):
    print(f"\n{BOLD}{BLUE}=== {text} ==={RESET}")

def run_cmd(cmd):
    try:
        full_cmd = f"source /opt/ros/humble/setup.bash && source /open_agbot_ws/install/setup.bash && {cmd}"
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.STDOUT).decode().strip()
    except:
        return None

header("SERIAL HARDWARE DEEP PROBE")

# Check if we can actually read strings from the Controller
print(f"{YELLOW}[?] Sniffing ACM1 (Controller) for text...{RESET}")
sniff_acm1 = subprocess.run("timeout 2 strings /dev/ttyACM1 | head -n 5", shell=True, capture_output=True).stdout.decode()
if sniff_acm1:
    print(f"    {GREEN}[✓] Raw Data Found: {sniff_acm1.splitlines()[0]}{RESET}")
else:
    print(f"    {RED}[✗] No readable text on ACM1. Hardware may be silent or wrong baud.{RESET}")

header("ROS TOPIC DEFINITIONS")
# Check if the message types are even known to the system
msg_check = run_cmd("ros2 interface show sensor_msgs/msg/NavSatFix")
if msg_check:
    print(f"{GREEN}[✓] ROS 2 Message definitions loaded.{RESET}")
else:
    print(f"{RED}[✗] ROS 2 Message definitions MISSING. Sourcing issue.{RESET}")

header("TOPIC PUBLISHER STATUS")
# Find out who is supposed to be talking
topics = ["/ublox_gps_node/fix", "/v_batt", "/motor_status"]
for t in topics:
    info = run_cmd(f"ros2 topic info {t}")
    if info:
        pubs = [line for line in info.split('\n') if "Publisher count" in line]
        print(f"{BOLD}{t}{RESET}: {pubs[0] if pubs else 'No Info'}")

header("LOG ERROR SCAN")
# Check the last 10 lines of ROS logs for 'Error' or 'Fail'
logs = run_cmd("ros2 doctor --report | grep 'log file:'")
print(f"{YELLOW}Check logs for hardware timeouts if publishers are 0.{RESET}")

header("DIAGNOSTIC COMPLETE")
