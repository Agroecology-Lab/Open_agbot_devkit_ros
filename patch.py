import os, subprocess, rclpy, threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, BatteryState
from nicegui import ui

class BotUI(Node):
    def __init__(self):
        super().__init__('web_ui')
        self.state = {'lat':0.0, 'lon':0.0, 'v':0.0, 's':'No Fix'}
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.g_cb, 10)
        self.create_subscription(BatteryState, '/battery_state', self.b_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
    def g_cb(self, m):
        self.state['lat'], self.state['lon'] = m.latitude, m.longitude
        self.state['s'] = {4:'RTK Fixed', 5:'RTK Float'}.get(m.status.status, 'GPS Fix')
    def b_cb(self, m): self.state['v'] = m.voltage
    def drive(self, l, a):
        t = Twist(); t.linear.x, t.angular.z = float(l), float(a)
        self.pub.publish(t)

def main():
    if not rclpy.ok(): rclpy.init()
    n = BotUI()
    threading.Thread(target=lambda: rclpy.spin(n), daemon=True).start()
    ui.dark_mode(True)
    with ui.header().classes('justify-between items-center'):
        ui.label('AgBot Control v2.0').classes('text-xl font-bold')
        ui.badge().bind_text_from(n.state, 's').props('color=green')
    with ui.column().classes('w-full items-center p-4'):
        ui.joystick(color='blue', on_move=lambda e: n.drive(e.y*0.5, -e.x*1.0), on_end=lambda _: n.drive(0,0))
        with ui.card().classes('w-64 mt-4'):
            ui.label().bind_text_from(n.state, 'v', backward=lambda v: f'Battery: {v:.2f}V').classes('text-2xl font-mono')
            ui.label().bind_text_from(n.state, 'lat', backward=lambda v: f'LAT: {v:.7f}')
            ui.label().bind_text_from(n.state, 'lon', backward=lambda v: f'LON: {v:.7f}')
    ui.run(port=8080, reload=False, show=False, title='AgBot')

if __name__ == '__main__':
    p = subprocess.check_output("find /workspace/src/basekit_ui -name ui_node.py", shell=True).decode().strip().split('\n')[0]
    with open(p, 'w') as f:
        f.write(open(__file__).read().replace('import os, subprocess, rclpy, threading', 'import threading, rclpy'))
    print(f"✅ Patched {p}")
    os.system(f"python3 {p}")
