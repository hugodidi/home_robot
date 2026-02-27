#!/usr/bin/env python3
"""
Sequential/Random Waypoint Patrol Node.

This script navigates a robot through a list of waypoints either sequentially
or in random order. It handles interruptions via SIGTERM/SIGINT signals and
publishes events when waypoints are reached for voice announcements.

Usage:
    ros2 run home_robot execute_patrol [--random]
"""

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml
import time
import os
import math
import signal
import sys
import random

# Global variables for signal detection
stop_patrol = False
navigator_global = None
cmd_vel_pub_global = None


def signal_handler(signum, frame):
    """Safe signal handler that only sets a flag."""
    global stop_patrol
    print("\n⚠ Interruption signal received, stopping patrol...")
    stop_patrol = True


def read_waypoints(file_path: str) -> dict:
    """Read waypoints from YAML file.
    
    Args:
        file_path: Path to the waypoints YAML file
        
    Returns:
        Dictionary containing waypoints data
    """
    with open(file_path, 'r') as f:
        waypoints_data = yaml.safe_load(f)
    return waypoints_data


def main():
    """Main patrol execution function.
    
    Initializes ROS2 navigator, loads waypoints, and executes patrol
    sequence (sequential or random). Handles navigation results and
    publishes events for voice announcements.
    """
    global stop_patrol
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    # Check if random order is requested via command line argument
    random_order = '--random' in sys.argv
    
    rclpy.init()

    navigator = BasicNavigator()
    
    # Make navigator globally accessible for signal handler
    global navigator_global, cmd_vel_pub_global
    navigator_global = navigator
    
    # Create publisher for patrol events (for voice announcements)
    patrol_event_pub = navigator.create_publisher(String, 'patrol_events', 10)
    
    # Create cmd_vel publisher (to stop robot when signal is received)
    cmd_vel_pub_global = navigator.create_publisher(Twist, '/cmd_vel', 10)

    # NEW: Secure stop via ROS topic
    def stop_callback(msg):
        global stop_patrol
        print("\n🛑 Stop request received via ROS topic!")
        stop_patrol = True
        
    navigator.create_subscription(String, '/stop_patrol', stop_callback, 10)

    # Wait for Nav2 action server to become available
    print("Waiting for Nav2 to be ready...")
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    
    action_client = ActionClient(navigator, NavigateToPose, 'navigate_to_pose')
    timeout_start = time.time()
    while not action_client.wait_for_server(timeout_sec=1.0):
        if time.time() - timeout_start > 60:
            print("ERROR: Timeout waiting for Nav2 action server")
            rclpy.shutdown()
            return
        print("  Waiting for action server...")
    print("✓ Nav2 action server ready!")

    # Get waypoints path from installed package
    try:
        pkg_share = get_package_share_directory('home_robot')
        waypoints_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    except Exception:
        # Fallback to project root if package not installed
        print("Warning: Package not found in install, searching in project workspace...")
        project_root = os.environ.get("PROJECT_ROOT", "/home/hugo/Escritorio/home_robot")
        waypoints_path = os.path.join(project_root, 'ros2_ws/src/home_robot/config/waypoints.yaml')

    if not os.path.exists(waypoints_path):
        print(f"ERROR: waypoints.yaml not found at {waypoints_path}")
        return
        
    waypoints_dict = read_waypoints(waypoints_path)
    waypoints_list = list(waypoints_dict['waypoints'].items())

    if random_order:
        print("🎲 Random mode activated. Shuffling waypoints...")
        random.shuffle(waypoints_list)

    # Wait for costmaps to populate before starting navigation
    print("Waiting 5 seconds to stabilize costmaps...")
    for _ in range(50):  # 5 seconds in 0.1s steps
        if stop_patrol:
            print("⚠ Patrol canceled before starting")
            rclpy.shutdown()
            return
        time.sleep(0.1)
    
    print(f"\n🎯 Starting patrol with {len(waypoints_list)} waypoints")
    try:
        for idx, (name, wp) in enumerate(waypoints_list, 1):
            # Check if we should stop before sending next goal
            if stop_patrol:
                print("\n⚠ Patrol stopped by external signal")
                break
                
            print(f"\n{'='*60}")
            print(f"[{idx}/{len(waypoints_list)}] Next goal: {name}")
            print(f"{'='*60}")
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(wp['x'])
            goal_pose.pose.position.y = float(wp['y'])
            
            # Convert theta (yaw) to quaternion
            theta = float(wp['theta'])
            goal_pose.pose.orientation.z = math.sin(theta / 2.0)
            goal_pose.pose.orientation.w = math.cos(theta / 2.0)

            # Send goal to navigation stack
            navigator.goToPose(goal_pose)
            print(f"[DEBUG] Goal sent to Nav2, waiting for result...")

            # Wait for task completion - use isTaskComplete() instead of result_future.done()
            # because result_future remains "done" from the previous waypoint
            while not stop_patrol:
                # Get feedback from navigation
                feedback = navigator.getFeedback()
                if feedback:
                    # Display remaining distance if available
                    distance = feedback.distance_remaining
                    if distance > 0:
                        print(f"  Remaining distance: {distance:.2f}m", end='\r')
                
                # Check if task is complete
                if navigator.isTaskComplete():
                    print()  # New line after progress display
                    break
                    
                time.sleep(0.1)
            
            # If stop signal received, cancel navigation and exit
            if stop_patrol:
                print("\n⚠ Patrol interrupted, canceling current navigation...")
                navigator.cancelTask()
                break

            result = navigator.getResult()
            print(f"\n[DEBUG] Navigation result for {name}: {result}")
            
            if result == TaskResult.SUCCEEDED:
                print(f"\n✓ Goal {name} reached successfully!")
                # Publish event for voice_controller to announce
                try:
                    event_msg = String()
                    event_msg.data = f"llegada:{name}"
                    patrol_event_pub.publish(event_msg)
                    print(f"[DEBUG] Arrival event published: {event_msg.data}")
                    time.sleep(0.2)  # Allow time for message to be published
                except Exception as e:
                    print(f"[ERROR] Could not publish event: {e}")
                print(f"[DEBUG] Continuing to next waypoint...")
                # IMPORTANT: No break here, continue to next waypoint
                
            elif result == TaskResult.CANCELED:
                print(f"\n⚠ Navigation canceled by external command.")
                break  # Exit patrol if canceled
                
            elif result == TaskResult.UNKNOWN:
                # If unknown and stop_patrol is set, it means we must exit NOW
                if stop_patrol:
                    print(f"\n⚠ Unknown result with stop flag, exiting patrol.")
                    break
                print(f"\n⚠ Unknown result for {name}, continuing to next...")
                # If not stopping, we can try next
                
            elif result == TaskResult.FAILED:
                print(f"\n❌ Failed to reach goal {name}. Stopping patrol.")
                break
            
            else:
                if stop_patrol:
                    break
                print(f"\n⚠ Unexpected result: {result}, continuing to next...")
            
            # Brief pause between waypoints
            print(f"[DEBUG] Waypoint {name} completed, continuing to next...")
            time.sleep(0.5)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\n🛑 Patrol interrupted.")
    finally:
        # Final cleanup: Execute only if ROS is still valid
        if rclpy.ok():
            print("  ⚠ Cleaning up navigation and stopping robot...")
            try:
                navigator.cancelTask()
                stop_msg = Twist()
                for _ in range(10):
                    cmd_vel_pub_global.publish(stop_msg)
                    time.sleep(0.02)
            except:
                pass
        
        if not rclpy.ok():
             print("\n🏁 Patrol finished (ROS shut down).")
        else:
             print("\n🏁 Patrol finished.")
             rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Ctrl-C detected, shutting down patrol...")
        rclpy.shutdown()
