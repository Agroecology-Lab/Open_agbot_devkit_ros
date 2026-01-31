Topological Navigation (AOC Branch)

This package implements a topological navigation layer on top of ROS 2 Nav2. It manages a graph-based representation of the environment where nodes represent semantic waypoints (e.g., "Field_Entry", "Row_1_Start") and edges define the navigable paths and actions between them.

The AOC (Area of Concern) branch specifically optimizes for agricultural and large-scale outdoor environments by utilizing a standalone YAML/TMap format.
Core Components

    map_manager2.py: The central node that loads and publishes the topological map. It manages the pointset (collection of nodes) and handles runtime map updates.

    navigation_executor.py: The "brain" that receives high-level node targets, plans a route through the graph, and interfaces with the Nav2 controller to execute movement.

    visualise_map.py: A utility node that converts the internal topological graph into visualization_msgs/MarkerArray for display in RViz.

Map Format: .tmap (YAML)

The AOC branch uses a structured YAML format. Unlike standard metric maps, a .tmap defines:

    Nodes: Named positions with X, Y, Z and QX, QY, QZ, QW orientation.

    Edges: Connections between nodes, including the action to perform (e.g., Maps_to_pose) and config parameters for the controller.

    Verifications: Specific checks to confirm the robot has successfully reached a node.

Example Node Definition
YAML

- node_name: "Waypoint1"
  pointset: "agbot_fields"
  pose:
    position:
      x: 10.5
      y: -2.0
      z: 0.0
    orientation:
      w: 1.0
  edges:
    - edge_id: "Waypoint1_to_Waypoint2"
      node: "Waypoint2"
      action: "navigate_to_pose"

Launching the Package

In the Sowbot ecosystem, the navigation stack is typically brought up via the basekit_launch.py.
Basic Execution
Bash

ros2 launch topological_navigation topological_navigation.launch.py \
    map_file:=/workspace/maps/test_map.yaml \
    pointset:=agbot_fields

Integrated Launch (Python Snippet)
Python

ld.add_action(Node(
    package="topological_navigation",
    executable="map_manager2.py",
    name="map_manager",
    arguments=[tmap_path],
    parameters=[{
        "tmap_file": tmap_path,
        "use_sim_time": is_sim,
        "pointset": "agbot_fields" 
    }],
    output='screen'
))

Usage
1. Monitoring Localization

The system publishes the current topological location on the /current_node topic:
Bash

ros2 topic echo /current_node

2. Sending a Navigation Goal

Navigation is handled via a ROS 2 Action. Instead of providing coordinates, you provide the node_name:
Bash

ros2 action send_goal /goto_node topological_navigation_msgs/action/GotoNode "{node: 'Waypoint2'}"

3. Visualizing in RViz

Add a MarkerArray display and subscribe to:

    /topological_map_visualisation (Nodes)

    /topological_map_edges_visualisation (Edges)

AOC Branch Specifics

    No Database Requirement: This branch eliminates the need for a running MongoDB instance. The .tmap file is the absolute source of truth.

    Improved Edge Actions: Supports passing custom YAML configurations directly to the Nav2 controller on a per-edge basis, allowing for different speeds or planners between specific waypoints.
