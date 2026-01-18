import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nicegui import app, ui

class AgBotUI(Node):
    def __init__(self):
        super().__init__('web_ui')
        self.lat = 0.0
        self.lon = 0.0
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_cb, 10)

    def gps_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

def main():
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = AgBotUI()

    @ui.page('/')
    def index():
        ui.dark_mode().enable()
        ui.label('AgBot Live Telemetry').classes('text-h4 mb-4')
        with ui.card().classes('w-64'):
            ui.label('GPS Status').classes('text-subtitle1 text-grey')
            ui.label().bind_text_from(ros_node, 'lat', backward=lambda x: f'Lat: {x:.6f}')
            ui.label().bind_text_from(ros_node, 'lon', backward=lambda x: f'Lon: {x:.6f}')
        
        # Move the timer inside the page or app context
        ui.timer(0.1, lambda: rclpy.spin_once(ros_node, timeout_sec=0))

    ui.run(port=8080, show=False, reload=False, title="AgBot")

if __name__ in {"__main__", "__mp_main__"}:
    main()
