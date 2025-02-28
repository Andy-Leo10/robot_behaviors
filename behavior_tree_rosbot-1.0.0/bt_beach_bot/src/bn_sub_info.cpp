#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include <std_msgs/msg/string.hpp>

using namespace BT;

class ReceiveString : public RosTopicSubNode<std_msgs::msg::String>
{
public:
    ReceiveString(const std::string &name, const NodeConfig &conf,
                  const RosNodeParams &params)
        : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("topic_name", "my_topic", "Topic to publish on"),
            OutputPort<std::string>("message", "Message received from the topic"),
        });
    }

    NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String> &last_msg) override
    {
        attempt_count_++; // Increment the attempt counter

        if (last_msg) // Check if a new message has been received
        {
            RCLCPP_INFO(logger(), "[%s] new message: %s (Attempt %d)", name().c_str(), last_msg->data.c_str(), attempt_count_);
            setOutput("message", last_msg->data); // Set the output message
            return NodeStatus::SUCCESS;
        }
        else
        {
            // RCLCPP_WARN(logger(), "[%s] no new message received (Attempt %d)", name().c_str(), attempt_count_);
            return NodeStatus::FAILURE;
        }
    }

    bool latchLastMessage() const override
    {
        return true; // Ensure the last message is retained until a new one is received
    }       

private:
    int attempt_count_; // Counter to track the number of attempts      
};