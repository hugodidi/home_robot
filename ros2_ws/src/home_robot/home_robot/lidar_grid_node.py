#!/usr/bin/env python3
"""
LiDAR Grid Node - Discretizes /scan into 8x8 grid and sends via Serial to Arduino.

This node subscribes to laser scan data, converts it to a discretized 8x8 occupancy
grid, and transmits the grid to an Arduino via serial communication. The robot is
assumed to be at the center of the grid.

Grid Orientation:
    - Row 0 = front of robot, Row 7 = back
    - Col 0 = right side, Col 7 = left side
    - Robot position: center between cells [3,3], [3,4], [4,3], [4,4]

Serial Protocol:
    - Distance message: "D:{min_dist:.2f}\n"
    - Grid message: "M:{hex1},{hex2},...,{hex8}\n"
    - Each hex byte represents one row (8 bits for 8 columns)

Usage:
    ros2 run home_robot lidar_grid_node --port /dev/ttyUSB0 --baud 115200 --grid 0.25
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import argparse
import sys


class LidarGridNode(Node):
    """
    ROS2 node that converts LiDAR scans to 8x8 occupancy grid for Arduino visualization.
    
    Subscribes to /scan topic and publishes grid data via serial port at ~5 Hz.
    """
    
    def __init__(self, port: str, baud: int, cell_size: float):
        """
        Initialize the LiDAR grid node.
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0' or '/dev/null' for testing)
            baud: Baud rate for serial communication (typically 115200)
            cell_size: Size of each grid cell in meters (e.g., 0.25m)
        """
        super().__init__('lidar_grid_node')
        
        self.cell_size = cell_size
        self.grid_size = 8
        # Total coverage = grid_size * cell_size (e.g., 8 * 0.25 = 2m)
        self.half_grid = (self.grid_size / 2) * self.cell_size
        
        # Serial setup (skip if /dev/null for testing)
        self.serial_enabled = port != '/dev/null'
        self.ser = None
        if self.serial_enabled:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f"Serial opened on {port} @ {baud} baud")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open {port}: {e}")
                self.serial_enabled = False
        else:
            self.get_logger().info("Test mode: serial disabled (/dev/null)")
        
        # Rate limiting (~5 Hz)
        self.last_send_time = self.get_clock().now()
        self.send_interval_ns = 200_000_000  # 200ms = 5Hz
        
        # Subscribe to /scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f"Grid: {self.grid_size}x{self.grid_size}, cell: {self.cell_size}m, coverage: {self.half_grid*2}m")

    def scan_callback(self, msg: LaserScan) -> None:
        """
        Process incoming laser scan and generate occupancy grid.
        
        Args:
            msg: LaserScan message from /scan topic
        """
        # Rate limiting
        now = self.get_clock().now()
        elapsed = (now - self.last_send_time).nanoseconds
        if elapsed < self.send_interval_ns:
            return
        self.last_send_time = now
        
        # Initialize 8x8 grid (all zeros)
        grid = [[0] * self.grid_size for _ in range(self.grid_size)]
        
        min_distance = float('inf')
        angle = msg.angle_min
        
        for r in msg.ranges:
            # Filter invalid ranges
            if math.isnan(r) or math.isinf(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue
            
            # Track minimum distance
            if r < min_distance:
                min_distance = r
            
            # Convert polar to Cartesian (robot frame: X forward, Y left)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # Map to grid cell
            # Robot is at center between cells [3,3], [3,4], [4,3], [4,4]
            # Grid X: 0 = front (+half_grid), 7 = back (-half_grid)
            # Grid Y: 0 = right (-half_grid), 7 = left (+half_grid)
            
            # Check if point is within grid bounds
            if abs(x) < self.half_grid and abs(y) < self.half_grid:
                # Convert to grid indices (0-7)
                # Row: front (x=+max) -> row 0, back (x=-max) -> row 7
                # Col: left (y=+max) -> col 0, right (y=-max) -> col 7
                # (Inverted to correct 180° rotation)
                row = int((x + self.half_grid) / self.cell_size)
                col = int((self.half_grid - y) / self.cell_size)
                
                # Clamp to valid range
                row = max(0, min(7, row))
                col = max(0, min(7, col))
                
                grid[row][col] = 1
            
            angle += msg.angle_increment
        
        # Convert grid to hex bytes
        # 90° counterclockwise rotation: new[7-col][row] = old[row][col]
        rotated_grid = [[0] * self.grid_size for _ in range(self.grid_size)]
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                rotated_grid[7 - col][row] = grid[row][col]
        
        hex_bytes = []
        for row in reversed(rotated_grid):  # 180°: reverse rows
            byte_val = 0
            for col, val in enumerate(row):
                if val:
                    byte_val |= (1 << (7 - col))  # 180°: reverse columns
            hex_bytes.append(f"{byte_val:02X}")
        
        # Handle case where no valid ranges were found
        if min_distance == float('inf'):
            min_distance = 10.0  # Default safe value
        
        # Build messages
        dist_msg = f"D:{min_distance:.2f}\n"
        grid_msg = f"M:{','.join(hex_bytes)}\n"
        
        # Log for debugging
        self.get_logger().debug(f"Min dist: {min_distance:.2f}m | Grid: {','.join(hex_bytes)}")
        
        # Send via serial
        if self.serial_enabled and self.ser and self.ser.is_open:
            try:
                self.ser.write(dist_msg.encode())
                self.ser.write(grid_msg.encode())
            except serial.SerialException as e:
                self.get_logger().warn(f"Serial error: {e}")
        else:
            # Print to console for testing
            self.get_logger().info(f"[TEST] {dist_msg.strip()} | {grid_msg.strip()}")

    def cleanup(self) -> None:
        """
        Send reset data to Arduino before closing.
        
        Clears the display by sending safe distance and empty grid.
        """
        if self.serial_enabled and self.ser and self.ser.is_open:
            try:
                self.get_logger().info("Sending reset to Arduino...")
                self.ser.write(b"D:10.00\n")  # Safe distance
                self.ser.write(b"M:00,00,00,00,00,00,00,00\n")  # Empty grid
                self.ser.flush()
            except Exception as e:
                self.get_logger().warn(f"Error sending reset: {e}")

    def destroy_node(self):
        self.cleanup()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    """
    Main entry point for the LiDAR grid node.
    
    Parses command-line arguments and runs the node until interrupted.
    """
    parser = argparse.ArgumentParser(description='LiDAR Grid Node for Arduino')
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('-g', '--grid', type=float, default=0.25, help='Cell size in meters (default: 0.25)')
    
    # Parse known args to allow ROS args to pass through
    parsed_args, remaining = parser.parse_known_args()
    
    rclpy.init(args=remaining)
    node = LidarGridNode(parsed_args.port, parsed_args.baud, parsed_args.grid)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Ctrl-C detected, shutting down lidar_grid_node...")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
