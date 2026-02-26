#!/usr/bin/env python3
"""
Navigation Service Node - Provides /go_to_pose service for robot navigation.

This node exposes a ROS2 service that accepts pose goals and forwards them
to Nav2's navigation stack via BasicNavigator.

Service:
    /go_to_pose (nav2_msgs/srv/SetInitialPose) - Send robot to target pose

Usage:
    ros2 run home_robot navigation_service
    
Note:
    This is a "fire-and-forget" service that returns immediately after
    sending the goal to Nav2. Use action servers for feedback/cancellation.
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import SetInitialPose
import math


class NavigationService(Node):
    """
    ROS2 service node for simple pose-based navigation commands.
    
    Wraps Nav2's BasicNavigator to provide a synchronous service interface
    for sending navigation goals.
    """
    
    def __init__(self):
        """Initialize the navigation service and BasicNavigator."""
        super().__init__('navigation_service_node')
        self.nav = BasicNavigator()
        
        # Create service using SetInitialPose message type for navigation
        # (reusing existing message type for simplicity)
        self.srv = self.create_service(
            SetInitialPose, 
            'go_to_pose', 
            self.go_to_pose_callback
        )
        self.get_logger().info('Service /go_to_pose ready (Type: nav2_msgs/srv/SetInitialPose)')

    def go_to_pose_callback(self, request, response):
        """
        Handle navigation goal requests.
        
        Args:
            request: SetInitialPose service request containing target pose
            response: Service response (empty, success implied)
            
        Returns:
            response: Empty response indicating goal was accepted
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Extract pose from request (SetInitialPose -> PoseWithCovarianceStamped -> Pose)
        goal_pose.pose = request.pose.pose.pose
        
        self.get_logger().info(f"Navigating to: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")
        
        # Send goal to Nav2
        self.nav.goToPose(goal_pose)
        
        # Note: This is a fire-and-forget service. Nav2 manages goal execution independently.
        # To wait for completion, add a loop checking self.nav.isTaskComplete()
        
        return response


def main():
    """Main entry point for the navigation service node."""
    rclpy.init()
    node = NavigationService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Ctrl-C detected, shutting down navigation_service...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
