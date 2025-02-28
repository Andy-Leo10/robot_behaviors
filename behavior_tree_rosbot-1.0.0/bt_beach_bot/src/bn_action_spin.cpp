#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/spin.hpp>

using namespace BT;
using SpinNav2 = nav2_msgs::action::Spin;
using GoalHandleSpinNav2 = rclcpp_action::ClientGoalHandle<SpinNav2>;

class Spin : public RosActionNode<SpinNav2>
{
public:
    Spin(const std::string &name,
             const NodeConfiguration &conf,
             const RosNodeParams &params)
        : RosActionNode<SpinNav2>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("action_name", "action_service", "Action server name"),
            InputPort<double>("target_yaw", "0.0", "Target yaw angle"),
            InputPort<int>("time_allowance_sec", "0", "Time allowance in seconds"),
        });
    }

protected:
    bool setGoal(SpinNav2::Goal &goal)
    {
        auto target_yaw = getInput<double>("target_yaw");
        auto time_allowance_sec = getInput<int>("time_allowance_sec");
        goal.target_yaw = target_yaw.value();
        goal.time_allowance.sec = time_allowance_sec.value();

        RCLCPP_INFO(logger(), "[%s]: Sending goal: target_yaw = %.2f, time_allowance = %ds", 
                    name().c_str(), goal.target_yaw, goal.time_allowance.sec);
        return true;
    }

    void onHalt() override
    {
        RCLCPP_INFO(logger(), "[%s]: onHalt", name().c_str());
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const nav2_msgs::action::Spin::Feedback> feedback) override
    {
        static int feedback_count = 0;
        feedback_count++;
        if (feedback_count % 20 == 0)
        {
            RCLCPP_INFO(logger(), "[%s]: feedback: angular_distance_traveled = %.2f", name().c_str(), feedback->angular_distance_traveled);
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
