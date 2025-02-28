#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h> // for tf2::Quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // for tf2::fromMsg

using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GoToPose : public RosActionNode<NavigateToPose>
{
public:
    GoToPose(const std::string &name,
             const NodeConfiguration &conf,
             const RosNodeParams &params)
        : RosActionNode<NavigateToPose>(name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("action_name", "action_service", "Action server name"),
            InputPort<double>("x"),
            InputPort<double>("y"),
            InputPort<double>("theta"),
        });
    }

protected:
    bool setGoal(NavigateToPose::Goal &goal)
    {
        double x, y, theta;
        if (!getInput("x", x) || !getInput("y", y) || !getInput("theta", theta))
        {
            return false;
        }

        goal.pose.header.frame_id = "map";
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        q.normalize();
        goal.pose.pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(logger(), "[%s]: Sending goal: x=%.2f, y=%.2f, theta=%.2f",
                    name().c_str(), x, y, theta);
        return true;
    }

    void onHalt() override
    {
        RCLCPP_INFO(logger(), "[%s]: onHalt", name().c_str());
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) override
    {
        static int feedback_count = 0;
        feedback_count++;
        if (feedback_count % 40 == 0)
        {
            double roll, pitch, yaw;
            tf2::Quaternion q;
            tf2::fromMsg(feedback->current_pose.pose.orientation, q);
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            double yaw_degrees = yaw * 180.0 / M_PI;

            RCLCPP_INFO(logger(), "[%s]: Current position: x=%.2f, y=%.2f, theta=%.2f°", 
                        name().c_str(), feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y, yaw_degrees);
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
