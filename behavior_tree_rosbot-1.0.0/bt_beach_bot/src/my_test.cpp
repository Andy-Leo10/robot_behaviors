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
        RCLCPP_INFO(logger(), "[%s]: feedback: angular_distance_traveled = %.2f", name().c_str(), feedback->angular_distance_traveled);
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

const char *string_xml = R"(
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Spin name="SpinX" action_name="spin" target_yaw="6.66" time_allowance_sec="11"/>
    </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_action_client_node");

    BehaviorTreeFactory factory;

    RosNodeParams params;
    params.nh = node;

    // Register the node type with the factory using the correct overload
    factory.registerNodeType<Spin>("Spin", params);

    // Load the behavior tree from a file
    auto tree = factory.createTreeFromText(string_xml);
    // auto tree = factory.createTreeFromFile("/home/asimovo/assets/behavior_tree_rosbot-1.0.0/bt_beach_bot/bt_xml/action_nav2.xml");

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