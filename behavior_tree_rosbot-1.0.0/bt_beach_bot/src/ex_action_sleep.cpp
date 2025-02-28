#include <behaviortree_ros2/bt_action_node.hpp>
#include "btcpp_ros2_interfaces/action/sleep.hpp"
#include "bn_action_sleep.cpp"
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
    params.server_timeout = std::chrono::seconds(10); 

    // Register the node type with the factory using the correct overload
    factory.registerNodeType<SleepAction>("SleepAction", params);

    // Load the behavior tree from a file
    auto tree = factory.createTreeFromFile("/home/asimovo/assets/behavior_tree_rosbot-1.0.0/bt_beach_bot/bt_xml/action_sleep.xml");

    NodeStatus status = NodeStatus::RUNNING;
    std::chrono::milliseconds custom_sleep_time(100);
    while(status == BT::NodeStatus::RUNNING)
    {
        rclcpp::spin_some(node);
        /*
        This method is used to tick the behavior tree once. It will tick the 
        root node of the tree a single time. This allows the behavior tree to 
        be evaluated and react to changes in the environment or the state of 
        the robot at each iteration of the loop.
        */
        status = tree.tickOnce();

        /*
        This function makes the current thread sleep for the specified duration 
        (custom_sleep_time). In the context of the main loop, it is used to 
        control the frequency of the ticks. By sleeping for a certain duration, 
        you can control how often the behavior tree is evaluated.
        */
        std::this_thread::sleep_for(custom_sleep_time);

        if (status == BT::NodeStatus::SUCCESS) RCLCPP_INFO(node->get_logger(), "Finished with status SUCCESS");
        else if (status == BT::NodeStatus::FAILURE) RCLCPP_INFO(node->get_logger(), "Finished with status FAILURE");
        // else RCLCPP_INFO(node->get_logger(), "Still RUNNING...");

    }

    rclcpp::shutdown();
    return 0;
}