#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/wait.hpp>

using namespace BT;
using WaitNav2 = nav2_msgs::action::Wait;
using GoalHandleWaitNav2 = rclcpp_action::ClientGoalHandle<WaitNav2>;

class Wait : public RosActionNode<WaitNav2>
{
public:
    Wait(const std::string &name,
             const NodeConfiguration &conf,
             const RosNodeParams &params)
        : RosActionNode<WaitNav2>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("action_name", "action_service", "Action server name"),
            InputPort<int>("time_sec", "0", "Time allowance in seconds"),
        });
    }

protected:
    bool setGoal(WaitNav2::Goal &goal)
    {
        auto time = getInput<int>("time_sec");
        goal.time.sec = time.value();

        RCLCPP_INFO(logger(), "[%s]: Sending goal: time = %ds", 
                    name().c_str(), goal.time.sec);
        return true;
    }

    void onHalt() override
    {
        RCLCPP_INFO(logger(), "[%s]: onHalt", name().c_str());
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const nav2_msgs::action::Wait::Feedback> feedback) override
    {
        static int feedback_count = 0;
        feedback_count++;
        if (feedback_count % 20 == 0)
        {
            RCLCPP_INFO(logger(), "[%s]: feedback: time_left = %ds", name().c_str(), feedback->time_left.sec);
        }        
        return NodeStatus::RUNNING;
    }

    BT::NodeStatus onResultReceived(const RosActionNode::WrappedResult &wr) override
    {
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(logger(), "[%s]: Goal reached successfully.", name().c_str());
            return NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(logger(), "[%s]: Failed to reach goal.", name().c_str());
            return NodeStatus::FAILURE;
        }
    }

    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "[%s]: onFailure with error: %s", name().c_str(), toStr(error));
        return NodeStatus::FAILURE;
    }
};
