#!/usr/bin/env python3
import os
import subprocess
import re
import time
import curses

# --- Configuration ---
EXPECTED_NODES = ["/basekit_driver_node", "/ublox_gps_node", "/basekit_ui_node"]
PRIORITY_TOPICS = {
    "/ublox_gps_node/fix": "GPS Fix",
    "/battery_state": "Battery",
    "/odom": "Odometry",
    "/cmd_vel": "Cmd Vel"
}
SERIAL_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]

def run_ros_cmd(cmd):
    try:
        full_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "[ -f /open_agbot_ws/install/setup.bash ] && "
            "source /open_agbot_ws/install/setup.bash; "
            f"{cmd}"
        )
        return subprocess.check_output(
            full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.DEVNULL
        ).decode().strip()
    except:
        return None

def get_serial_details():
    details = {}
    for dev in SERIAL_PORTS:
        if os.path.exists(dev):
            try:
                fuser_out = subprocess.check_output(["fuser", dev], stderr=subprocess.DEVNULL).decode().strip()
                pid = fuser_out.split()[-1] if fuser_out else None
                details[dev] = {"status": "BUSY", "pid": pid} if pid else {"status": "AVAILABLE", "pid": None}
            except Exception:
                details[dev] = {"status": "AVAILABLE", "pid": None}
        else:
            details[dev] = {"status": "MISSING", "pid": None}
    return details

def draw_uber_dashboard(stdscr):
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_RED, -1)
    curses.init_pair(3, curses.COLOR_CYAN, -1)
    curses.init_pair(4, curses.COLOR_YELLOW, -1)
    
    curses.curs_set(0)
    stdscr.nodelay(True)  # Make getch non-blocking
    stdscr.timeout(100)   # Wait 100ms for input, then continue

    last_update = 0
    hw_info = {}
    nodes_raw = ""
    all_topics = ""
    topic_stats = {}

    while True:
        # Only update heavy data every 2 seconds to keep UI responsive
        current_time = time.time()
        if current_time - last_update > 2.0:
            hw_info = get_serial_details()
            nodes_raw = run_ros_cmd("ros2 node list") or ""
            all_topics = run_ros_cmd("ros2 topic list") or ""
            
            for topic in PRIORITY_TOPICS:
                if topic in all_topics:
                    hz_raw = run_ros_cmd(f"timeout 0.5s ros2 topic hz {topic}")
                    rate = re.findall(r"average rate: ([\d.]+)", hz_raw) if hz_raw else None
                    topic_stats[topic] = f"{rate[0]} Hz" if rate else "Streaming..."
                else:
                    topic_stats[topic] = "Offline"
            last_update = current_time

        stdscr.clear()
        height, width = stdscr.getmaxyx()

        if height < 18 or width < 50:
            stdscr.addstr(0, 0, "WINDOW TOO SMALL", curses.color_pair(2))
        else:
            try:
                # Header
                stdscr.attron(curses.color_pair(3) | curses.A_BOLD)
                stdscr.addstr(0, 0, " 🚀 AGBOT MISSION CONTROL ".center(width, "="))
                stdscr.attroff(curses.color_pair(3) | curses.A_BOLD)

                # Hardware
                stdscr.addstr(2, 2, "SERIAL HARDWARE", curses.A_UNDERLINE)
                for i, (dev, data) in enumerate(hw_info.items()):
                    row = 3 + i
                    if data["status"] == "AVAILABLE":
                        stdscr.addstr(row, 4, "[✓]", curses.color_pair(1))
                        stdscr.addstr(row, 8, f"{dev}: Available")
                    elif data["status"] == "BUSY":
                        stdscr.addstr(row, 4, "[❗]", curses.color_pair(4))
                        stdscr.addstr(row, 8, f"{dev}: BUSY (PID {data['pid']})", curses.color_pair(2))
                    else:
                        stdscr.addstr(row, 4, "[✗]", curses.color_pair(2))
                        stdscr.addstr(row, 8, f"{dev}: NOT FOUND")

                # Nodes
                stdscr.addstr(7, 2, "ACTIVE NODES", curses.A_UNDERLINE)
                for i, node in enumerate(EXPECTED_NODES):
                    color = curses.color_pair(1) if node in nodes_raw else curses.color_pair(2)
                    char = "[✓]" if node in nodes_raw else "[✗]"
                    stdscr.addstr(8 + i, 4, char, color)
                    stdscr.addstr(8 + i, 8, f"{node}")

                # Topics
                stdscr.addstr(12, 2, "TOPIC HEALTH", curses.A_UNDERLINE)
                row = 13
                for topic, label in PRIORITY_TOPICS.items():
                    status_val = topic_stats.get(topic, "Loading...")
                    color = curses.color_pair(1) if "Hz" in status_val or "Stream" in status_val else curses.color_pair(2)
                    stdscr.addstr(row, 4, f"{label:<12} | {status_val}", color)
                    row += 1

                # Footer
                footer_text = f" Press 'q' to Exit | Last Update: {time.strftime('%H:%M:%S')} "
                stdscr.addstr(height-1, 0, footer_text.center(width, "-"))
            except curses.error:
                pass

        stdscr.refresh()
        
        # Capture input
        key = stdscr.getch()
        if key == ord('q') or key == ord('Q'):
            break

if __name__ == "__main__":
    try:
        curses.wrapper(draw_uber_dashboard)
    except KeyboardInterrupt:
        pass
