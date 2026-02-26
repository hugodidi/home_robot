#!/usr/bin/env python3
"""
Overhead Camera Service Node - Opens rqt_image_view for ceiling camera feed.

This node provides a ROS2 service to launch an image viewer window displaying
the overhead camera feed. Prevents duplicate windows.

Service:
    /overhead_cam (std_srvs/srv/Trigger) - Open camera viewer window

Parameters:
    topic (string): Image topic to subscribe (default: /overhead_cam)

Usage:
    ros2 run home_robot overhead_cam_service
    ros2 service call /overhead_cam std_srvs/srv/Trigger
"""

import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class OverheadCamService(Node):
    """
    Service node that launches rqt_image_view for overhead camera visualization.
    
    Manages a subprocess running rqt_image_view and ensures only one instance
    is active at a time.
    """
    
    def __init__(self):
        """Initialize the overhead camera service."""
        super().__init__("overhead_cam_service")

        self.declare_parameter("topic", "/overhead_cam")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

        self.srv = self.create_service(
            Trigger, "overhead_cam", self._handle_request
        )
        self._process: subprocess.Popen | None = None

        self.get_logger().info(
            f"Overhead-cam service ready  →  topic: {self.topic}"
        )

    def _handle_request(self, _request, response):
        """
        Handle service request to open camera viewer.
        
        Args:
            _request: Trigger request (empty)
            response: Trigger response with success status and message
            
        Returns:
            response: Populated with success status and message
        """
        # Prevent duplicate viewer windows
        if self._process is not None and self._process.poll() is None:
            response.success = False
            response.message = "Viewer window already open."
            self.get_logger().warn(response.message)
            return response

        try:
            # Launch rqt_image_view with topic remapping
            self._process = subprocess.Popen(
                [
                    "ros2", "run", "rqt_image_view", "rqt_image_view",
                    "--ros-args", "-r", f"image:={self.topic}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            response.success = True
            response.message = (
                f"Overhead camera viewer opened on {self.topic}"
            )
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to open viewer: {e}"
            self.get_logger().error(response.message)

        return response

    def destroy_node(self):
        """Clean up by terminating viewer subprocess."""
        # Terminate rqt_image_view if still running
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
        super().destroy_node()


def main(args=None):
    """Main entry point for the overhead camera service node."""
    rclpy.init(args=args)
    node = OverheadCamService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Ctrl-C detected, shutting down overhead_cam_service...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
