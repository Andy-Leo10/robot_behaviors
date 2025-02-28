#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <std_msgs/msg/string.hpp>

using namespace BT;

class SendString : public RosTopicPubNode<std_msgs::msg::String>
{
public:
    SendString(const std::string& name, 
            const NodeConfiguration& conf,
            const RosNodeParams& params)
        : RosTopicPubNode<std_msgs::msg::String>(name, conf, params) 
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("topic_name", "my_topic", "Topic to publish on"),
            InputPort<std::string>("msg", "Message to publish")
        });
    }

protected:
    bool setMessage(std_msgs::msg::String &msg) override
    {
        auto message = getInput<std::string>("msg");
        if (!message)
        {
            throw RuntimeError("Missing required input [msg]");
        }
        msg.data = message.value();
        // RCLCPP_INFO(node_->get_logger(), "Sending message: %s", msg.data.c_str());
        RCLCPP_INFO(node_->get_logger(), "[%s] Sending message: %s", name().c_str(), msg.data.c_str());
        return true;
    }
};
