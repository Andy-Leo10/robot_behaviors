#include <behaviortree_ros2/bt_action_node.hpp>
#include "btcpp_ros2_interfaces/action/sleep.hpp"

using namespace BT;

class SleepAction : public RosActionNode<btcpp_ros2_interfaces::action::Sleep>
{
public:
    SleepAction(const std::string &name,
                const NodeConfig &conf,
                const RosNodeParams &params)
        : RosActionNode<btcpp_ros2_interfaces::action::Sleep>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("action_name", "action_service", "Action server name"),
            InputPort<unsigned>("msec")
            });
    }

    bool setGoal(Goal &goal) override
    {
        auto timeout = getInput<unsigned>("msec");
        goal.msec_timeout = timeout.value();
        RCLCPP_INFO(logger(), "[%s]: Sending goal: timeout = %d", name().c_str(), goal.msec_timeout);
        return true;
    }

    void onHalt() override
    {
        RCLCPP_INFO(logger(), "[%s]: onHalt", name().c_str());
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const btcpp_ros2_interfaces::action::Sleep::Feedback> feedback) override
    {
        static int feedback_count = 0;
        feedback_count++;
        if (feedback_count % 5 == 0)
        {
            RCLCPP_INFO(logger(), "[%s]: feedback: %d", name().c_str(), feedback->cycle);
        }
        return NodeStatus::RUNNING;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
        RCLCPP_INFO(logger(), "[%s]: onResultReceived. Done = [%s]", name().c_str(),
                    wr.result->done ? "true" : "false");

        return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "[%s]: onFailure with error: [%s]", name().c_str(), toStr(error));
        return NodeStatus::FAILURE;
    }
};
