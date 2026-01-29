import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
import tf2_geometry_msgs
import yaml
import numpy as np
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from topological_navigation_msgs.msg import TopologicalOccupiedNode, TopologicalRoute
from geometry_msgs.msg import Point, Pose
from action_msgs.msg import GoalStatusArray

# ===============================================================
#  UTILS
# ===============================================================               
def load_waypoints_from_tmap(file_path):
    """
    Loads and parses a tmap file, and pre-transforms the local vertices of each
    waypoint into the global 'map' frame.
    """
    try:
        with open(file_path, 'r') as file:
            tmap_data = yaml.safe_load(file)
            
        waypoints_data = {}
        for node_entry in tmap_data.get('nodes', []):
            try:
                node_name = node_entry['node']['name']
                pos = node_entry['node']['pose']['position']
                orient = node_entry['node']['pose']['orientation']

                # Create the transformation matrix for this waypoint
                translation = [pos['x'], pos['y'], pos['z']]
                rotation = [orient['x'], orient['y'], orient['z'], orient['w']]
                    
                trans_matrix = tf_transformations.translation_matrix(translation)
                rot_matrix = tf_transformations.quaternion_matrix(rotation)
                transform_matrix = np.dot(trans_matrix, rot_matrix)

                # Transform local vertices into map frame vertices
                local_verts = node_entry['node'].get('verts', [])
                map_frame_verts = []
                for vert in local_verts:
                    # Represent local vertex as a 4D point for matrix multiplication
                    local_point = np.array([vert['x'], vert['y'], 0, 1])
                    map_point = np.dot(transform_matrix, local_point)
                    # Store the transformed (x, y) coordinates
                    map_frame_verts.append((map_point[0], map_point[1]))

                # Store the pre-transformed data
                waypoints_data[node_name] = {
                    'position': (pos['x'], pos['y'], pos['z']),
                    'orientation': (orient['x'], orient['y'], orient['z'], orient['w']),
                    'verts': map_frame_verts
                }
                    
            except KeyError as e:
                continue
            
        return waypoints_data

    except Exception as e:
        return {}
    
# ==============================================================================
# Route Visualiser Node
# ==============================================================================
class RouteVisualiserNode(Node):
    """
    Subscribes to a list of waypoint names and publishes visualization markers.
    """
    def __init__(self):
        """ Initializes the Route Visualiser Node."""
        super().__init__('route_visualiser')
        self.declare_parameter('tmap', rclpy.Parameter.Type.STRING)
        self.tmap_path = self.get_parameter('tmap').get_parameter_value().string_value
        self.tmap = load_waypoints_from_tmap(self.tmap_path)

        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.routevis_pub = self.create_publisher(MarkerArray, 'topological_route_visualisation', qos_profile=latching_qos)

        self.create_subscription(TopologicalRoute, "topological_navigation/Route", self.route_cb, qos_profile=latching_qos)
        self.create_subscription(TopologicalRoute, "topological_navigation/Route", self.route_cb, qos_profile=latching_qos)
        # TODO: it would be nice to clear the route when the action completes, but the status topic is unreliable
        # self.create_subscription(GoalStatusArray, "/topological_navigation/_action/status", self.status_callback, qos_profile=latching_qos)
        # self.create_subscription(GoalStatusArray, "/topological_navigation/execute_policy_mode/_action/status", self.status_callback, qos_profile=latching_qos)
        self.get_logger().info('Route Visualiser node has started.')
        
    
    # TODO: it would be nice to clear the route when the action completes, but the status topic is unreliable
    # def status_callback(self, msg: GoalStatusArray):
    #     """
    #     Checks the status of all goals and clears the route if any have finished.
    #     """
    #     # Define terminal states: SUCCEEDED, CANCELED, ABORTED
    #     terminal_states = [4, 5, 6]
    #     status = msg.status_list[-1]
    #     if status.status in terminal_states:
    #         self.get_logger().info("Action finished. Clearing route visualization.")
    #         self.clear_route()


    def route_cb(self, msg):
        """ Callback for receiving a new route. """
        self.clear_route() # TODO: it would be nice to clear the route when the action completes, but the status topic is unreliable
        self.route_marker = MarkerArray()
        self.route_marker.markers=[]
        idn = 0
        if self.tmap is not None:
            for i in range(1,len(msg.nodes)):
                marker = self.get_route_marker(msg.nodes[i-1], msg.nodes[i], idn)
                self.route_marker.markers.append(marker)
                idn+=1
            self.routevis_pub.publish(self.route_marker)


    def clear_route(self):
        """ Clears the current route visualisation. """
        self.route_marker = MarkerArray()
        self.route_marker.markers=[]
        marker = Marker()
        marker.action = marker.DELETEALL
        self.route_marker.markers.append(marker)
        self.routevis_pub.publish(self.route_marker) 


    def get_route_marker(self, origin, end, idn):
        """ Creates a marker between two waypoints. """
        marker = Marker()
        marker.id = idn
        marker.header.frame_id = 'topo_map'
        marker.type = marker.ARROW
        origin_node = self.tmap[origin]
        end_node = self.tmap[end]
        V1 = Point(x=origin_node['position'][0], y=origin_node['position'][1], z=origin_node['position'][2])
        V1.z += 0.25
        V2 = Point(x=end_node['position'][0], y=end_node['position'][1], z=end_node['position'][2])
        V2.z += 0.25
        marker.pose.orientation.w= 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 0.33
        marker.color.g = 0.99
        marker.color.b = 0.55
        marker.points.append(V1)
        marker.points.append(V2)
        marker.ns='/route_pathg'
        return marker
    
    
# ==============================================================================
#  Occupied Visualiser Node
# ==============================================================================
class OccupancyVisualiserNode(Node):
    """
    Subscribes to the list of occupied waypoints and publishes visualization markers.
    """
    def __init__(self):
        """ Initializes the Occupancy Visualiser Node. """
        super().__init__('occupancy_visualiser')
        self.declare_parameter('tmap', rclpy.Parameter.Type.STRING)
        self.tmap_path = self.get_parameter('tmap').get_parameter_value().string_value
        self.tmap = load_waypoints_from_tmap(self.tmap_path)

        self.last_marker_count = 0
        
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, '/topological_navigation/visual/occupied_node', latching_qos)

        self.create_subscription(TopologicalOccupiedNode, '/topological_navigation/occupied_node', self.occupancy_cb, 10)
        self.get_logger().info('Waypoint Visualiser component has started.')

    def occupancy_cb(self, msg):
        """ Callback for receiving the list of currently occupied waypoints. """
        waypoint_names = msg.nodes if msg.nodes else []
        marker_array = MarkerArray()

        # Create ADD markers for all currently occupied waypoints
        for i, wp_name in enumerate(waypoint_names):
            wp_data = self.tmap.get(wp_name)
            if not wp_data: continue

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "occupied_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            pose_coords = wp_data['position']
            marker.pose.position.x = pose_coords[0]
            marker.pose.position.y = pose_coords[1]
            marker.pose.position.z = 0.15  # Lift it off the ground slightly
            marker.pose.orientation.w = 1.0

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.1

            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8  # Slightly transparent
            
            marker_array.markers.append(marker)

        # Create DELETE markers for any old markers that are no longer needed
        for i in range(len(waypoint_names), self.last_marker_count):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "occupied_waypoints"
            marker.id = i
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.last_marker_count = len(waypoint_names)

# ===============================================================
#  The Main Execution Block
# ===============================================================
def main(args=None):
    rclpy.init(args=args)

    route_visualiser_node = RouteVisualiserNode()
    occupancy_visualiser_node = OccupancyVisualiserNode()

    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    
    # Add nodes to the executor
    executor.add_node(route_visualiser_node)
    executor.add_node(occupancy_visualiser_node)

    try:
        # Spin the executor to run both nodes
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        route_visualiser_node.destroy_node()
        occupancy_visualiser_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()