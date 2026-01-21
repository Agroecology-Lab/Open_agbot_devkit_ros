import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from nicegui import ui
import threading

class AgbotInterface(Node):
    def __init__(self):
        super().__init__('basekit_ui_node')
        self.marker = None
        self.map_obj = None
        self.status_badge = None
        self.first_fix = False
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

    def set_ui_targets(self, marker, map_obj, status_badge):
        self.marker = marker
        self.map_obj = map_obj
        self.status_badge = status_badge

    def gps_callback(self, msg):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
            
        # Update Status Badge
        if self.status_badge:
            # logic for ZED-F9P status mapping
            if msg.status.status == 2:
                self.status_badge.set_text('RTK FIXED')
                self.status_badge.classes(replace='bg-green-6')
            elif msg.status.status == 1:
                self.status_badge.set_text('RTK FLOAT')
                self.status_badge.classes(replace='bg-orange-6')
            elif msg.status.status == 0:
                self.status_badge.set_text('3D FIX')
                self.status_badge.classes(replace='bg-blue-6')
            else:
                self.status_badge.set_text('NO FIX')
                self.status_badge.classes(replace='bg-red-6')

        # Move Marker
        if self.marker and msg.status.status >= 0:
            self.marker.move(msg.latitude, msg.longitude)
            if not self.first_fix:
                self.map_obj.set_center((msg.latitude, msg.longitude))
                self.first_fix = True
            else:
                self.map_obj.run_map_method('panTo', [msg.latitude, msg.longitude])

    def publish_drive(self, x, y):
        msg = Twist()
        msg.linear.x = float(y) * -1.0  
        msg.angular.z = float(x) * -1.0 
        self.cmd_pub.publish(msg)

if not rclpy.ok(): rclpy.init()
interface = AgbotInterface()

@ui.page('/')
def index():
    ui.dark_mode().enable()
    ui.add_head_html('<style>.nicegui-joystick { background: #1d1d1d !important; border: 2px solid #444 !important; border-radius: 50% !important; }</style>')

    with ui.column().classes('w-full items-center p-4'):
        ui.label('Open Agbot Control Panel').classes('text-h4 text-white q-mb-md')
        
        with ui.row().classes('w-full justify-center gap-6 no-wrap'):
            # LEFT: Status
            with ui.column().classes('w-64'):
                with ui.card().classes('w-full bg-grey-9 p-4'):
                    ui.label('SYSTEM STATUS').classes('text-caption text-grey-5')
                    ui.label('Connected').classes('text-green text-bold text-lg')
                
                ui.label('ACTIVE TOPICS').classes('text-caption text-grey-5 q-mt-md q-ml-sm')
                with ui.card().classes('w-full p-2 bg-grey-10 border-grey-8'):
                    for t in ['/v_bat', '/i_mot', '/gps/fix', '/cmd_vel']:
                        with ui.row().classes('items-center gap-2 p-1'):
                            ui.icon('radio_button_checked', size='xs', color='blue')
                            ui.label(t).classes('text-xs font-mono text-grey-4')

            # CENTER: Map with RTK Status Overlay
            with ui.column().classes('flex-grow max-w-3xl'):
                with ui.card().classes('w-full h-[450px] p-0 overflow-hidden shadow-2xl border border-grey-8 relative'):
                    m = ui.leaflet(center=(20, 0), zoom=3).classes('w-full h-full')
                    m.tile_layer(
                        url_template='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                        options={'attribution': 'Esri Satellite'}
                    )
                    marker = m.marker(latlng=(0, 0))
                    
                    # RTK Status Badge Overlay
                    with ui.row().classes('absolute-top-right q-ma-md z-[1000]'):
                        gps_badge = ui.label('WAITING...').classes('p-2 rounded text-white text-bold bg-grey-7 shadow-md')
                    
                    interface.set_ui_targets(marker, m, gps_badge)
                    ui.label('SATELLITE VIEW').classes('absolute-top-left q-ma-sm text-xs z-[1000] text-white bg-black/50 p-1 rounded')

                ui.button('EMERGENCY STOP', on_click=lambda: ui.notify('E-STOP ENGAGED', color='red')) \
                    .classes('w-full q-mt-md py-4 text-white bg-red-9 text-bold text-lg shadow-lg')

            # RIGHT: Joystick
            with ui.column().classes('w-72 items-center'):
                with ui.card().classes('p-8 bg-grey-9 items-center w-full'):
                    ui.label('JOYSTICK').classes('text-caption text-grey-5 q-mb-xl')
                    ui.joystick(
                        color='white',
                        size=180,
                        on_move=lambda e: interface.publish_drive(e.x, e.y),
                        on_end=lambda _: interface.publish_drive(0, 0)
                    )
                    ui.label('Drag to Drive').classes('q-mt-xl text-grey-5 text-xs')

    if not hasattr(interface, 'thread'):
        interface.thread = threading.Thread(target=lambda: rclpy.spin(interface), daemon=True)
        interface.thread.start()

def main(args=None):
    ui.run(title='Agbot UI', port=8080, reload=False, dark=True)

if __name__ == '__main__':
    main()
