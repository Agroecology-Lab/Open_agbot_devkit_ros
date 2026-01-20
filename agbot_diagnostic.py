import os
import subprocess
import time
import psutil

def get_styling(status):
    if status == "ALIVE": return "\033[92m[✓]\033[0m" # Green
    if status == "ZOMBIE": return "\033[93m[?]\033[0m" # Yellow
    return "\033[91m[X]\033[0m" # Red

def check_system_resources():
    print("\n--- [1/4] System Load Analysis ---")
    cpu_load = psutil.cpu_percent(interval=0.5)
    mem = psutil.virtual_memory()
    print(f"Total CPU Load: {cpu_load}% | Memory Usage: {mem.percent}%")
    
    procs = sorted(psutil.process_iter(['pid', 'name', 'cpu_percent']), 
                   key=lambda x: x.info['cpu_percent'], reverse=True)[:3]
    for p in procs:
        print(f"  > PID {p.info['pid']}: {p.info['cpu_percent']}% - {p.info['name']}")

def check_serial_load():
    print("\n--- [2/4] Serial Port Health ---")
    ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
    for port in ports:
        if os.path.exists(port):
            # Check for multi-process conflicts
            conflicts = []
            for proc in psutil.process_iter(['pid', 'name']):
                try:
                    for f in proc.open_files():
                        if port in f.path:
                            conflicts.append(f"{proc.info['name']} ({proc.info['pid']})")
                except (psutil.AccessDenied, psutil.NoSuchProcess):
                    continue
            
            conflict_msg = f" | \033[91mCONFLICTS: {', '.join(conflicts)}\033[0m" if conflicts else " | Port Clean"
            
            # Bandwidth Test
            cmd = f"timeout 1s cat {port} | wc -c"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            bps = int(result.stdout.strip() or 0)
            print(f"  {port}: {bps} bytes/sec {conflict_msg}")
        else:
            print(f"  {port}: NOT FOUND")

def scan_all_topics():
    print("\n--- [3/4] Dynamic ROS 2 Topic Scan ---")
    try:
        raw_list = subprocess.check_output(['ros2', 'topic', 'list'], text=True)
        topics = [line.strip() for line in raw_list.split('\n') if line.strip()]
        
        print(f"Found {len(topics)} topics. Checking activity...")
        
        for topic in topics:
            if any(x in topic for x in ["parameter_events", "rosout"]):
                continue

            try:
                # 1-second listen for data
                cmd = ['ros2', 'topic', 'echo', topic, '--once']
                subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=1.0)
                print(f"  {get_styling('ALIVE')} {topic.ljust(45)} (Data Flowing)")
            except subprocess.TimeoutExpired:
                print(f"  {get_styling('ZOMBIE')} {topic.ljust(45)} (No Heartbeat)")
            except Exception:
                print(f"  {get_styling('ERROR')} {topic.ljust(45)} (Error)")

    except Exception as e:
        print(f"Error listing topics: {e}")

def check_dds_config():
    print("\n--- [4/4] Network/DDS Environment ---")
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    rmw = os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    print(f"  ROS_DOMAIN_ID: {domain}")
    print(f"  RMW: {rmw}")

if __name__ == "__main__":
    print("\n" + "="*50)
    print("         AGBOT DEEP DIAGNOSTIC TOOL")
    print("="*50)
    check_system_resources()
    check_serial_load()
    check_dds_config()
    scan_all_topics()
    print("\n" + "="*50)
    print("DIAGNOSTIC COMPLETE")
    print
