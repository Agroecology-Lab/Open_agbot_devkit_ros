#!/usr/bin/env python3
import os, subprocess, re, time, curses

# --- Configuration ---
EXPECTED_NODES = ["/basekit_driver_node", "/ublox_gps_node", "/basekit_ui_node"]
TARGET_TOPICS = ["/fix", "/diagnostics", "/odom", "/cmd_vel"]
SERIAL_MAP = {"/dev/ttyACM0": "ESP32 MCU", "/dev/ttyACM2": "ZED-F9P GPS"}

def run_cmd(cmd):
    try:
        full_cmd = f"source /opt/ros/humble/setup.bash && [ -f /open_agbot_ws/install/setup.bash ] && source /open_agbot_ws/install/setup.bash; {cmd}"
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.DEVNULL).decode().strip()
    except: return None

def draw(stdscr):
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_RED, -1)
    curses.init_pair(3, curses.COLOR_CYAN, -1)
    curses.curs_set(0); stdscr.nodelay(True); stdscr.timeout(100)
    
    topic_data = {}
    while True:
        nodes = run_cmd("ros2 node list") or ""
        topics = run_cmd("ros2 topic list") or ""
        stdscr.clear()
        stdscr.addstr(0, 0, "🚀 AGBOT MISSION CONTROL (INTERNAL)", curses.color_pair(3) | curses.A_BOLD)
        
        # --- Nodes Section ---
        stdscr.addstr(2, 0, "NODES STATUS", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in nodes
            color = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(3+i, 2, f"{'[✓]' if exists else '[✗]'} {node}", color)

        # --- Topics Section ---
        stdscr.addstr(8, 0, "TELEMETRY (Hz)", curses.A_UNDERLINE)
        for i, topic in enumerate(TARGET_TOPICS):
            stat = "Offline"
            if topic in topics:
                hz = run_cmd(f"timeout 0.4s ros2 topic hz {topic}")
                stat = re.findall(r"average rate: ([\d.]+)", hz)[0] + " Hz" if hz and "average" in hz else "Streaming..."
            
            color = curses.color_pair(1) if "Hz" in stat or "Stream" in stat else curses.color_pair(2)
            stdscr.addstr(9+i, 2, f"{topic:<15}: {stat}", color)
            topic_data[topic] = stat
            
        stdscr.addstr(15, 0, "Press 'q' to exit and see summary report", curses.A_DIM)
        stdscr.refresh()
        if stdscr.getch() == ord('q'): 
            return {"nodes": nodes, "topics": topic_data}

if __name__ == "__main__":
    final = curses.wrapper(draw)
    print("\n" + "="*45)
    print("📋 FINAL MISSION REPORT")
    print("="*45)
    
    print(f"\n[SYSTEM NODES]")
    for node in EXPECTED_NODES:
        status = "✅ ACTIVE" if node in final['nodes'] else "❌ OFFLINE"
        print(f"  {node:<20} : {status}")

    print(f"\n[TOPIC HEALTH]")
    for topic, hz in final['topics'].items():
        icon = "🟢" if "Hz" in hz or "Stream" in hz else "🔴"
        print(f"  {icon} {topic:<15} : {hz}")

    print(f"\n[HARDWARE MAPPING]")
    for port, name in SERIAL_MAP.items():
        exists = os.path.exists(port)
        print(f"  {'🔗' if exists else '🚫'} {port:<12} -> {name} ({'CONNECTED' if exists else 'DISCONNECTED'})")
    
    print("\n" + "="*45 + "\n")
