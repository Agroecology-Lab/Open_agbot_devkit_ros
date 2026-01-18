#!/bin/bash
echo "Starting Lizard Simulation... Press Ctrl+C to stop."
while true; do
  # Simulate 13.5V Battery
  ros2 topic pub -1 /battery_state sensor_msgs/msg/BatteryState "{voltage: 13.5}" > /dev/null 2>&1
  
  # Simulate Clear Bumper
  ros2 topic pub -1 /bumper_front_top_state std_msgs/msg/Bool "{data: false}" > /dev/null 2>&1
  
  # Simulate 0.5 m/s Speed
  ros2 topic pub -1 /odom nav_msgs/msg/Odometry "{twist: {twist: {linear: {x: 0.5}}}}" > /dev/null 2>&1
  
  sleep 1
done
