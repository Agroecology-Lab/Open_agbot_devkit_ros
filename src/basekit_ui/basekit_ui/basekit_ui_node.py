import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nicegui import ui
import threading

class BasekitUI(Node):
    def __init__(self):
        super().__init__('web_ui')
        self.state = {'gps_status': 'Waiting...', 'lat': 51.539424, 'lon': -0.118092, 
                      'volt': 0.0, 'perc': 0.0, 'estop_active': False}
        self.first_fix = False

        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, 10)
        self.create_subscription(BatteryState, '/battery_state', self.bat_cb, 10)
        self.create_subscription(Bool, '/estop1_state', lambda m: self.set_s('estop_active', m.data), 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

    def set_s(self, k, v): self.state[k] = v

    def gps_cb(self, msg):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        status_map = {-1: 'No Fix', 0: '3D Fix', 1: 'RTK Float', 2: 'RTK FIXED'}
        current_status = status_map.get(msg.status.status, 'Unknown')
        self.state.update({'lat': msg.latitude, 'lon': msg.longitude, 'gps_status': current_status})
        if hasattr(self, 'map'):
            if not self.first_fix:
                self.map.set_center((msg.latitude, msg.longitude))
                self.first_fix = True
            else:
                self.map.run_map_method('panTo', [msg.latitude, msg.longitude])
            self.marker.move(msg.latitude, msg.longitude)

    def bat_cb(self, msg):
        self.state.update({'volt': msg.voltage, 'perc': msg.percentage})
    
    def drive(self, x, y):
        t = Twist()
        t.linear.x = float(y) * 0.5 
        t.angular.z = float(x) * -1.0
        self.cmd_pub.publish(t)

    def stop_robot(self):
        self.stop_pub.publish(Bool(data=True))
        ui.notify('Emergency Stop!', type='negative')

def main(args=None):
    if not rclpy.ok(): rclpy.init(args=args)
    node = BasekitUI()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    @ui.page('/')
    def index():
        ui.dark_mode().enable()
        ui.add_head_html('<style>.nicegui-joystick { background: #1d1d1d !important; border: 2px solid #444 !important; border-radius: 50% !important; }</style>')
        
        with ui.header().classes('items-center justify-between bg-grey-10'):
            ui.label('AgBot Control').classes('text-h5 text-green-500')
            ui.badge().bind_text_from(node.state, 'gps_status').classes('p-2')

        with ui.row().classes('w-full no-wrap p-4'):
            with ui.column().classes('flex-grow'):
                with ui.card().classes('w-full p-0 overflow-hidden border border-grey-8'):
                    node.map = ui.leaflet(center=(node.state['lat'], node.state['lon']), zoom=18).classes('h-[500px] w-full')
                    node.map.tile_layer(url_template='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}')
                    node.marker = node.map.marker(latlng=(node.state['lat'], node.state['lon']))

            with ui.column().classes('w-80 gap-4'):
                with ui.card().classes('w-full items-center bg-grey-9'):
                    ui.label('JOYSTICK').classes('text-caption text-grey-5')
                    ui.joystick(color='white', size=160, on_move=lambda e: node.drive(e.x, e.y), on_end=lambda _: node.drive(0, 0))
                with ui.card().classes('w-full bg-grey-9'):
                    ui.button('STOP', on_click=node.stop_robot).classes('bg-red-9 text-white w-full h-16 text-bold')
                with ui.card().classes('w-full bg-grey-9'):
                    ui.linear_progress().bind_value_from(node.state, 'perc')
                    ui.label().bind_text_from(node.state, 'volt', backward=lambda x: f"{x:.2f}V").classes('text-center w-full')

    ui.run(title='Agbot Dashboard', port=8080, show=False, reload=False)

if __name__ == '__main__':
    main()
