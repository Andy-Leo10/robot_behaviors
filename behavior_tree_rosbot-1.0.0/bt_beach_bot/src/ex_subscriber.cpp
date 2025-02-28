#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/string.hpp>
#include "bn_sub_info.cpp"

using namespace BT;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("subscriber_test");

    BehaviorTreeFactory factory;

    RosNodeParams params;
    params.nh = node;
    // params.default_port_value = "btcpp_string";
    factory.registerNodeType<ReceiveString>("ReceiveString", params);

    auto tree = factory.createTreeFromFile("/home/asimovo/assets/behavior_tree_rosbot-1.0.0/bt_beach_bot/bt_xml/subscriber.xml");

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
            RCLCPP_INFO(node->get_logger(), "\033[1;32mFinished with status SUCCESS\033[0m");
        else if (status == BT::NodeStatus::FAILURE)
            RCLCPP_ERROR(node->get_logger(), "Finished with status FAILURE");
        else
            RCLCPP_INFO(node->get_logger(), "Still RUNNING...");
    }
    rclcpp::shutdown();
    return 0;
}

// publish msgs on that topic
// ros2 topic pub /my_topic std_msgs/msg/String "data: 'Hello, World'" -1
