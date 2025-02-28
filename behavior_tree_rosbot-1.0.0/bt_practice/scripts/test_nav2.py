#!/usr/bin/env python3

"""
Script that tests the `navigate_to_pose` action client to move the robot base.

Example usage:
  Python:   python3 test_nav2.py --x 1 --y 1 --theta 1.57
  ros2:     ros2 run bt_practice test_nav2.py --x 1 --y 1 --theta 1.57
"""

import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import transforms3d
from nav2_msgs.action import NavigateToPose


class Nav2Client(Node):
    def __init__(self):
        super().__init__("nav2_client")
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Nav2 test node started")

    def send_pose_goal(self, x, y, theta):
        self.get_logger().info(f"Going to [x: {x}, y: {y}, theta: {theta}] ...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        quat = transforms3d.euler.euler2quat(0, 0, theta)
        goal_msg.pose.pose.orientation.w = quat[0]
        goal_msg.pose.pose.orientation.x = quat[1]
        goal_msg.pose.pose.orientation.y = quat[2]
        goal_msg.pose.pose.orientation.z = quat[3]
        self.cli.wait_for_server()
        send_goal_future = self.cli.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_position = feedback.current_pose.pose.position
        current_orientation = feedback.current_pose.pose.orientation
        distance_remaining = feedback.distance_remaining        
        # self.get_logger().info(f"Received feedback: {feedback}")
        self.get_logger().info(f"Current Position: x={current_position.x}, y={current_position.y}")
        # self.get_logger().info(f"Current Orientation: z={current_orientation.z}, w={current_orientation.w}")
        # self.get_logger().info(f"Distance Remaining: {distance_remaining}")

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Nav2 test script")
    parser.add_argument("--x", type=float, default=1.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--theta", type=float, default=0.0)
    args = parser.parse_args()  

    # Start ROS node and action client
    rclpy.init()
    client = Nav2Client()

    # Send goal to the navigate_to_pose action server
    client.send_pose_goal(args.x, args.y, args.theta)
    rclpy.spin(client)

'''
if problems:
sed -i 's/np.float/np.float64/g' /usr/lib/python3/dist-packages/transforms3d/quaternions.py
'''    
