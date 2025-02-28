#include <behaviortree_ros2/bt_service_node.hpp>
#include <std_srvs/srv/empty.hpp>

using namespace BT;

class ReinitializeLocalization : public RosServiceNode<std_srvs::srv::Empty>
{
public:
    ReinitializeLocalization(const std::string &name,
                                    const NodeConfig &conf,
                                    const RosNodeParams &params)
        : RosServiceNode<std_srvs::srv::Empty>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<std::string>("service_name", "/reinitialize_global_localization", "Service server name")
        });
    }

    bool setRequest(std_srvs::srv::Empty::Request::SharedPtr &request) override
    {
        (void)request;
        RCLCPP_INFO(logger(), "[%s] - Setting request", name().c_str());
        return true;
    }

    BT::NodeStatus onResponseReceived(const std_srvs::srv::Empty::Response::SharedPtr &response) override
    {
        (void)response;
        // this response is empty
        RCLCPP_INFO(logger(), "[%s] - Received response", name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "[%s]: onFailure with error: %s", name().c_str(), toStr(error));
        return BT::NodeStatus::FAILURE;
    }
};
