import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseArray, PoseStamped
import math
import tf2_ros
import tf2_geometry_msgs
import yaml
import numpy as np
import tf_transformations
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from topological_navigation_msgs.msg import TopologicalOccupiedNode

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
                    'pose': (pos['x'], pos['y']),
                    'verts': map_frame_verts
                }
                    
            except KeyError as e:
                continue
            
        return waypoints_data

    except Exception as e:
        return {}


# ===============================================================
#  NODE A: The Transformer Class
# ===============================================================
class PoseTransformerNode(Node):
    def __init__(self):
        super().__init__('pose_transformer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseArray, '/plan_in_map_frame', 10)
        self.create_subscription(PoseArray, '/rownav_teb_poses', self.plan_callback, 10)
        self.get_logger().info('Pose Transformer component has started.')

    def plan_callback(self, msg):
        """
        Transforms a PoseStamped from the odom frame to the map frame using TF2.
        """
        poses = []

        # Check if the transform is available before proceeding
        try:
            self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'Could not transform {msg.header.frame_id} to map: {ex}')
            return

        for pose_odom in msg.poses:

            pose_stamped_in = PoseStamped()
            pose_stamped_in.header = msg.header
            pose_stamped_in.pose = pose_odom

            try:
                timeout = rclpy.duration.Duration(seconds=0.1)
                transformed_pose_stamped = self.tf_buffer.transform(pose_stamped_in, 'map', timeout)
                poses.append(transformed_pose_stamped.pose)

            except tf2_ros.TransformException as ex:
                self.get_logger().error(f'Could not transform {msg.header.frame_id} to map: {ex}')
                continue

        # Publish the transformed poses
        if poses:
            msgnew = PoseArray()
            msgnew.header.frame_id = 'map'
            msgnew.header.stamp = msg.header.stamp
            msgnew.poses = poses
            self.publisher.publish(msgnew)


# ===============================================================
#  NODE B: The Checker Class
# ===============================================================
class OccupancyCheckerNode(Node):
    def __init__(self):
        super().__init__('occupancy_checker')
        
        # 1. DECLARE PARAMETERS for file path and threshold
        self.declare_parameter('tmap', rclpy.Parameter.Type.STRING)
        self.declare_parameter('occupancy_threshold', rclpy.Parameter.Type.DOUBLE)

        # 2. GET PARAMETER VALUES
        self.tmap_path = self.get_parameter('tmap').get_parameter_value().string_value
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').get_parameter_value().double_value

        # 3. subscribe to the '/plan_in_map_frame' topic.
        self.create_subscription(PoseArray, '/plan_in_map_frame', self.teb_poses_cb, 10)
        self.create_subscription(String, '/topological_navigation/current_destination', self.current_destination_cb, 10)

        # 4. Load waypoints from the specified topological map file
        self.tmap = load_waypoints_from_tmap(self.tmap_path)

        # 5. Create Publisher and State variable to only publish when the set of waypoints changes
        self.occupancy_pub = self.create_publisher(TopologicalOccupiedNode, '/topological_navigation/occupied_node', 10)
        self.last_published_wps = set()
        self.current_destination = None
        
        self.get_logger().info('Occupancy Checker has started.')
        if self.tmap:
            self.get_logger().info(f'Successfully loaded and now monitoring {len(self.tmap)} waypoints.')
        else:
            self.get_logger().warn('No waypoints loaded. Please check the topological map file path.')
            return
        self.get_logger().info(f'occupancy threshold is {self.occupancy_threshold} meters.')
    

    def teb_poses_cb(self, msg):
        """
        Callback to check if any received pose overlaps with waypoints.
        Uses a polygon check if vertices are available, otherwise uses a distance threshold.
        """
        occupied_wps = set()
        if self.current_destination is not None: occupied_wps.add(self.current_destination)
        for i, pose in enumerate(msg.poses):
            for wp, wp_data in self.tmap.items():
                is_occupied = False
                verts = wp_data.get('verts', [])
                if verts:
                    if self._is_inside_polygon(pose.position.x, pose.position.y, verts):
                        is_occupied = True
                        occupied_wps.add(wp)
                else:
                    self.get_logger().warn("Checking overlap using DISTANCE THRESHOLD.")
                    waypoint_pose = wp_data['pose']
                    distance = math.hypot(pose.position.x - waypoint_pose[0], pose.position.y - waypoint_pose[1])
                    
                    if distance < self.occupancy_threshold:
                        self.get_logger().warn(f"OCCUPANCY: Pose #{i} is NEAR Waypoint {wp} (using distance check).")
                        is_occupied = True
                        occupied_wps.add(wp)
                
                if is_occupied:
                    break

        if occupied_wps != self.last_published_wps:
            # Create the message object
            occupancy_msg = TopologicalOccupiedNode()
            occupancy_msg.nodes = list(occupied_wps)
            self.occupancy_pub.publish(occupancy_msg)

            # Log the change for debugging
            if occupancy_msg.nodes: self.get_logger().debug(f"Occupied nodes: {occupancy_msg.nodes}")

            # Update the state for the next check
            self.last_published_wps = occupied_wps
            
    def current_destination_cb(self, msg: String):
        self.current_destination = msg.data

    def _is_inside_polygon(self, point_x, point_y, polygon_verts):
        num_verts = len(polygon_verts)
        is_inside = False
        p1_x, p1_y = polygon_verts[0]
        for i in range(1, num_verts + 1):
            p2_x, p2_y = polygon_verts[i % num_verts] # Use modulo to wrap around to the first vertex
            if point_y > min(p1_y, p2_y):
                if point_y <= max(p1_y, p2_y):
                    if point_x <= max(p1_x, p2_x):
                        if p1_y != p2_y:
                            # Calculate the x-intersection of the line
                            x_intersection = (point_y - p1_y) * (p2_x - p1_x) / (p2_y - p1_y) + p1_x
                            if p1_x == p2_x or point_x <= x_intersection:
                                is_inside = not is_inside
                
            p1_x, p1_y = p2_x, p2_y
                
        return is_inside
    
    
# # ==============================================================================
# #  NODE C: The Visualiser Class
# # ==============================================================================
# class WaypointVisualiserNode(Node):
#     """
#     Subscribes to the list of occupied waypoints and publishes visualization markers.
#     """
#     def __init__(self):
#         super().__init__('occupied_waypoint_visualiser')
#         self.declare_parameter('tmap', rclpy.Parameter.Type.STRING)
#         self.tmap_path = self.get_parameter('tmap').get_parameter_value().string_value
#         self.tmap = load_waypoints_from_tmap(self.tmap_path)

#         self.last_marker_count = 0
        
#         latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
#         self.marker_pub = self.create_publisher(MarkerArray, '/topological_navigation/visual/occupied_node', latching_qos)

#         self.create_subscription(TopologicalOccupiedNode, '/topological_navigation/occupied_node', self.occupancy_cb, 10)
#         self.get_logger().info('Waypoint Visualiser component has started.')

#     def occupancy_cb(self, msg):
#         waypoint_names = msg.nodes if msg.nodes else []
#         marker_array = MarkerArray()

#         # Create ADD markers for all currently occupied waypoints
#         for i, wp_name in enumerate(waypoint_names):
#             wp_data = self.tmap.get(wp_name)
#             if not wp_data: continue

#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "occupied_waypoints"
#             marker.id = i
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
            
#             pose_coords = wp_data['pose']
#             marker.pose.position.x = pose_coords[0]
#             marker.pose.position.y = pose_coords[1]
#             marker.pose.position.z = 0.15  # Lift it off the ground slightly
#             marker.pose.orientation.w = 1.0

#             marker.scale.x = 1.0
#             marker.scale.y = 1.0
#             marker.scale.z = 0.1

#             marker.color.r = 1.0  # Red
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.color.a = 0.8  # Slightly transparent
            
#             marker_array.markers.append(marker)

#         # Create DELETE markers for any old markers that are no longer needed
#         for i in range(len(waypoint_names), self.last_marker_count):
#             marker = Marker()
#             marker.header.frame_id = "map"
#             marker.ns = "occupied_waypoints"
#             marker.id = i
#             marker.action = Marker.DELETE
#             marker_array.markers.append(marker)

#         self.marker_pub.publish(marker_array)
#         self.last_marker_count = len(waypoint_names)

# ===============================================================
#  The Main Execution Block
# ===============================================================
def main(args=None):
    rclpy.init(args=args)

    # Create instances of both nodes
    transformer_node = PoseTransformerNode()
    checker_node = OccupancyCheckerNode()
    # visualizer_node = WaypointVisualiserNode()

    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    
    # Add both nodes to the executor
    executor.add_node(transformer_node)
    executor.add_node(checker_node)
    # executor.add_node(visualizer_node)

    try:
        # Spin the executor to run both nodes
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        transformer_node.destroy_node()
        checker_node.destroy_node()
        # visualizer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()