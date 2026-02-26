#!/usr/bin/env python3
"""
Navigation Manager Node.

Manages sequential or random waypoint navigation for the robot.
This node loads waypoints from a YAML file and navigates through
them using Nav2 BasicNavigator.

Parameters:
    waypoints_file (str): Path to waypoints YAML file
    mode (str): Navigation mode - 'sequential' or 'random'
"""

import os
import yaml
import time
import random
import math
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class NavigationManager(Node):
    """Manages waypoint navigation for the home robot."""
    
    def __init__(self):
        """Initialize NavigationManager node with parameters and BasicNavigator."""
        super().__init__('navigation_manager')
        
        # Get default waypoints path from package
        pkg_share = get_package_share_directory('home_robot')
        default_wp_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
        
        # Declare dynamic parameters
        self.declare_parameter('waypoints_file', default_wp_path)
        self.declare_parameter('mode', 'sequential')  # sequential or random
        
        self.waypoints_path = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        self.navigator = BasicNavigator()
        
        # Load waypoints from file
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            self.get_logger().error(f"Could not load waypoints from {self.waypoints_path}")
            return

        # Wait for Nav2 to become active
        self.get_logger().info("Waiting for Nav2 to activate (BT Navigator)...")
        # In Jazzy with SLAM Toolbox, we avoid waiting for localizer as it's not a standard lifecycle node
        self.navigator.waitUntilNav2Active(localizer='robot_state_publisher') 
        self.get_logger().info("Nav2 ready.")

    def load_waypoints(self) -> dict:
        """Load waypoints from YAML file.
        
        Returns:
            Dictionary of waypoints with name as key and coordinates as value.
            Empty dict if file not found or parsing fails.
        """
        if not os.path.exists(self.waypoints_path):
            return {}
        try:
            with open(self.waypoints_path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', {})
        except Exception as e:
            self.get_logger().error(f"Error reading waypoints file: {e}")
            return {}

    def euler_to_quaternion(self, yaw: float) -> dict:
        """Convert Euler yaw angle to quaternion representation.
        
        Args:
            yaw: Yaw angle in radians
            
        Returns:
            Dictionary with 'z' and 'w' quaternion components
        """
        return {
            'z': math.sin(yaw / 2),
            'w': math.cos(yaw / 2)
        }

    def run(self):
        """Execute waypoint navigation in sequential or random order.
        
        Navigates through all loaded waypoints, monitoring progress and
        handling navigation results (success, cancel, failure).
        """
        wp_names = list(self.waypoints.keys())
        
        if self.mode == 'random':
            random.shuffle(wp_names)
            self.get_logger().info("Mode: Random")
        else:
            self.get_logger().info("Mode: Sequential")

        for name in wp_names:
            wp = self.waypoints[name]
            self.get_logger().info(f"Navigating to: {name} (x: {wp['x']}, y: {wp['y']})")
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(wp['x'])
            goal_pose.pose.position.y = float(wp['y'])
            
            q = self.euler_to_quaternion(float(wp['theta']))
            goal_pose.pose.orientation.z = q['z']
            goal_pose.pose.orientation.w = q['w']

            # Send robot to goal pose
            self.navigator.goToPose(goal_pose)

            # Monitor navigation progress
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    pass
                time.sleep(1.0)

            # Check navigation result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Reached {name}!")
                # self.speak(f"He llegado a {name}")  # Commented: announcement done by voice_controller if active
                # Pause between waypoints
                time.sleep(2.0)
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f"Mission canceled for {name}.")
                break
            elif result == TaskResult.FAILED:
                self.get_logger().error(f"Failed to navigate to {name}.")
                break

    def speak(self, text: str):
        """Log voice announcement (placeholder for TTS integration).
        
        Args:
            text: Text to be spoken
        """
        self.get_logger().info(f"[VOICE]: {text}")


def main(args=None):
    """Main entry point for navigation_manager node."""
    rclpy.init(args=args)
    manager = NavigationManager()
    if manager.waypoints:
        try:
            manager.run()
        except KeyboardInterrupt:
            manager.get_logger().info("\nCtrl-C detected, canceling navigation...")
            try:
                manager.navigator.cancelTask()
            except:
                pass
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
