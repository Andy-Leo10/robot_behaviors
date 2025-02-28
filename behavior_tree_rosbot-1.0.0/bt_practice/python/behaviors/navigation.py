"""
Navigation behaviors
"""

import py_trees
import transforms3d

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node


class GetLocationFromQueue(py_trees.behaviour.Behaviour):
    """Gets a location name from the queue"""

    def __init__(self, name, location_dict):
        super(GetLocationFromQueue, self).__init__(name)
        self.location_dict = location_dict
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        """Checks for the status of the navigation action"""
        loc_list = self.bb.get("loc_list")
        if loc_list:
            location_name = loc_list.pop(0)
            self.bb.set("current_location", self.location_dict[location_name])
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class GoToPose(py_trees.behaviour.Behaviour):
    """Sends a navigation goal to the nav2 action server"""

    def __init__(self, name, node: Node):
        super(GoToPose, self).__init__(name)
        self.node = node
        self._action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.bb = py_trees.blackboard.Blackboard()

    def initialise(self):
        """Initialise the action client and send the goal"""
        current_location = self.bb.get("current_location")
        if current_location:
            x, y, theta = current_location
            self.bb.set("nav_status", None)  # Initialize nav_status
            self.send_goal(x, y, theta)
        else:
            self.node.get_logger().info("No current location set in blackboard")

    def send_goal(self, x, y, theta):
        self.node.get_logger().info(f'\nSending goal to action server: x={x}, y={y}, theta={theta}')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        quat = transforms3d.euler.euler2quat(0, 0, theta)
        goal_pose.pose.pose.orientation.w = quat[0]
        goal_pose.pose.pose.orientation.x = quat[1]
        goal_pose.pose.pose.orientation.y = quat[2]
        goal_pose.pose.pose.orientation.z = quat[3]

        self.node.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.node.get_logger().info('Action server detected')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.node.get_logger().info('Goal sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            self.bb.set("nav_status", GoalStatus.STATUS_ABORTED)
            return
        self.node.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Result: {result}')
        self.bb.set("nav_status", GoalStatus.STATUS_SUCCEEDED if result else GoalStatus.STATUS_ABORTED)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_position = feedback.current_pose.pose.position
        current_orientation = feedback.current_pose.pose.orientation
        distance_remaining = feedback.distance_remaining
        # self.node.get_logger().info(f"Current Position: x={current_position.x}, y={current_position.y}")
        # self.node.get_logger().info(f"Current Orientation: z={current_orientation.z}, w={current_orientation.w}")
        # self.node.get_logger().info(f"Distance Remaining: {distance_remaining}")

    def update(self):
        """Check the status of the navigation action"""
        nav_status = self.bb.get("nav_status")
        if nav_status == GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        elif nav_status == GoalStatus.STATUS_ABORTED:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING