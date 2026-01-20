#!/bin/bash
WS_DIR="$HOME/open_agbot_ws"
VENV_PYTHON="$WS_DIR/venv/bin/python3"
LIZ_CONFIG="$WS_DIR/src/basekit_launch/config/basekit.liz"
PORT="/dev/ttyACM0"
[ -e /dev/ttyACM1 ] && PORT="/dev/ttyACM1"

echo "[*] Flashing MCU..."
"$VENV_PYTHON" -m esptool --chip esp32s3 --port "$PORT" --baud 921600 write-flash -z \
    0x0000 "$WS_DIR/src/basekit_driver/build/bootloader/bootloader.bin" \
    0x8000 "$WS_DIR/src/basekit_driver/build/partition_table/partition-table.bin" \
    0x10000 "$WS_DIR/src/basekit_driver/build/lizard.bin"

echo "[*] Entering Ultra-Safe Injection..."
sleep 5

"$VENV_PYTHON" -c "
import serial, time
try:
    # Open without any flow control toggles
    ser = serial.Serial('$PORT', 115200, timeout=1, rtscts=False, dsrdtr=False)
    
    # Wake up character
    ser.write(b'\n')
    time.sleep(2) # Long wait for USB-CDC to initialize
    ser.reset_input_buffer()
    
    # Filter config: No comments, no empty lines, no whitespace-only lines
    with open('$LIZ_CONFIG', 'r') as f:
        lines = [l.strip() for l in f if l.strip() and not l.strip().startswith('#') and not l.strip().startswith('//')]
    
    print(f'   > Cleaned config to {len(lines)} active lines.')
    
    for i, line in enumerate(lines):
        print(f'[{i+1}/{len(lines)}] Sending: {line}')
        ser.write((line + '\n').encode())
        ser.flush()
        time.sleep(0.2) # Delay per line
    
    ser.write(b'core.restart()\n')
    print('Done.')
except Exception as e: print(f'Error: {e}')
"
