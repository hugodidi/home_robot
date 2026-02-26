#!/usr/bin/env python3
"""
Odometry to TF Broadcaster Node.

This node subscribes to /odom topic and publishes the corresponding TF transform.
The frame IDs are extracted from the odometry message itself to avoid hardcoding.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """Node that publishes TF transforms from odometry messages."""
    
    def __init__(self):
        """Initialize the OdomToTF node."""
        super().__init__('odom_to_tf')
        
        # Do not hardcode child_frame_id; extract it from the /odom message
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.get_logger().info("Publishing TF from /odom (odom -> child_frame_id from msg)")

    def odom_callback(self, msg: Odometry) -> None:
        """Process odometry message and publish TF transform.
        
        Args:
            msg: Odometry message containing pose and frame information.
        """
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or "odom"
        t.child_frame_id = msg.child_frame_id or "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    """Main entry point for the odom_to_tf node."""
    rclpy.init()
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nCtrl-C detected, shutting down odom_to_tf...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
