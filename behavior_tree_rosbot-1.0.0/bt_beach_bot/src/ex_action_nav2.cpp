#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "bn_action_nav2.cpp"
#include <chrono> // for setting the frequency of the control loop

using namespace BT;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_action_client_node");

    BehaviorTreeFactory factory;

    RosNodeParams params;
    params.nh = node;
    
    /*
    This parameter sets the timeout duration for the action server.
    If the action server does not respond within this time frame,
    the action client will consider the goal as failed and will trigger the
    SEND_GOAL_TIMEOUT error. This is useful to avoid waiting indefinitely
    for a response from the action server.
    */
    params.server_timeout = std::chrono::seconds(45);

    // Register the node type with the factory using the correct overload
    factory.registerNodeType<GoToPose>("GoToPose", params);

    // Load the behavior tree from a file
    auto tree = factory.createTreeFromFile("/home/asimovo/assets/behavior_tree_rosbot-1.0.0/bt_beach_bot/bt_xml/action_nav2.xml");

    NodeStatus status = NodeStatus::RUNNING;
    std::chrono::milliseconds custom_sleep_time(10);
    while (status == BT::NodeStatus::RUNNING)
    {
        rclcpp::spin_some(node);
        /*
        This method is used to continuously tick the behavior tree while it is
        running. It will tick the root node of the tree at the specified
        interval (custom_sleep_time). This ensures that the behavior tree is
        evaluated periodically and can react to changes in the environment or
        the state of the robot.
        */
        status = tree.tickWhileRunning(custom_sleep_time);

        if (status == BT::NodeStatus::SUCCESS)
            RCLCPP_INFO(node->get_logger(), "Finished with status SUCCESS");
        else if (status == BT::NodeStatus::FAILURE)
            RCLCPP_INFO(node->get_logger(), "Finished with status FAILURE");
        else
            RCLCPP_INFO(node->get_logger(), "Still RUNNING...");
    }
    rclcpp::shutdown();
    return 0;
}