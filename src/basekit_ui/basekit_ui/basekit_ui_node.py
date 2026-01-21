import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from nicegui import ui, app
import threading

class BasekitUI(Node):
    def __init__(self):
        super().__init__('web_ui')
        
        # Internal state to hold the latest data for the UI
        self.state = {
            'latitude': 0.0,
            'longitude': 0.0,
            'gps_status': 'Waiting for Fix...',
            'speed': 0.0
        }

        # Subscriber for GPS data
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10
        )

    def gps_callback(self, msg):
        self.state['latitude'] = msg.latitude
        self.state['longitude'] = msg.longitude
        self.state['gps_status'] = 'Fixed' if msg.status.status >= 0 else 'Searching...'

def main(args=None):
    # 1. Initialize ROS 2
    rclpy.init(args=args)
    ui_node = BasekitUI()

    # 2. Run ROS 2 spinning in a separate thread 
    # This prevents ROS from blocking the NiceGUI web server
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ui_node), daemon=True)
    ros_thread.start()

    # 3. Build the NiceGUI Interface
    @ui.page('/')
    def index():
        ui.label('Open Agbot Control Panel').classes('text-h4 q-pa-md')
        
        with ui.row():
            with ui.card():
                ui.label('GPS Information').classes('text-h6')
                # Labels bind directly to the node's state
                ui.label().bind_text_from(ui_node.state, 'gps_status', backward=lambda x: f"Status: {x}")
                ui.label().bind_text_from(ui_node.state, 'latitude', backward=lambda x: f"Lat: {x:.6f}")
                ui.label().bind_text_from(ui_node.state, 'longitude', backward=lambda x: f"Lon: {x:.6f}")

            with ui.card():
                ui.label('System Actions').classes('text-h6')
                ui.button('Emergency Stop', on_click=lambda: ui.notify('E-STOP Pressed')).props('color=red')

    # 4. Start NiceGUI
    # show=False is important for Docker; reload=False prevents loop conflicts
    ui.run(title='Agbot UI', port=8080, show=False, reload=False)

    # Cleanup after UI is closed
    ui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
