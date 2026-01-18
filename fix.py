import os
import yaml
import subprocess
import time

CONFIG_PATH = 'src/ublox/ublox_gps/config/c94_m8p_rover.yaml'
PORT = '/dev/ttyACM0'
BAUD_RATES = [9600, 38400, 57600, 115200]

def hunt():
    print(f"--- Starting Baud Rate Hunt on {PORT} ---")
    
    found_baud = None
    for baud in BAUD_RATES:
        print(f"Testing {baud}...", end=" ", flush=True)
        
        # Set OS level baud rate
        subprocess.run(f"stty -F {PORT} {baud}", shell=True, capture_output=True)
        
        # Try to read raw data
        try:
            cmd = f"timeout 1.5s cat {PORT} | head -n 1"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            # Check if we got NMEA data (usually starts with $)
            if result.stdout and '$' in result.stdout:
                print(f" [SUCCESS!]")
                found_baud = baud
                break
            else:
                print(" [No Data]")
        except Exception:
            print(" [Error]")

    if found_baud:
        update_yaml(found_baud)
    else:
        print("\n[CRITICAL] Still no data found. Please:")
        print("1. Physically UNPLUG the GPS USB and plug it back in.")
        print("2. Ensure the GPS has a clear view of the sky (or is near a window).")

def update_yaml(baud):
    with open(CONFIG_PATH, 'r') as f:
        data = yaml.safe_load(f)
    
    # Update the baudrate in the nested structure
    data['ublox_gps_node']['ros__parameters']['uart1']['baudrate'] = baud
    data['ublox_gps_node']['ros__parameters']['set_baudrate'] = False
    data['ublox_gps_node']['ros__parameters']['config_on_startup'] = False
    
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    
    print(f"\n--- UPDATED YAML ---")
    print(f"The GPS is talking at {baud}. YAML has been updated.")
    print(f"You can now run: python3 openagbot-dev.py")

if __name__ == "__main__":
    hunt()
