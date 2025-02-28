// Vision related behavior

#include "vision_behavior.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <unistd.h>

using std::placeholders::_1;

// LOOKFOROBJECT
// Looks for an object of a certain color, specified by a parameter
LookForObject::LookForObject(const std::string& name,
                             const BT::NodeConfig& config,
                             rclcpp::Node::SharedPtr node_ptr) :
    BT::StatefulActionNode(name, config), node_ptr_{node_ptr}
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus LookForObject::onStart() {
    received_image_ = false;
    image_sub_ = image_transport::create_subscription(
        node_ptr_.get(), "/camera/color/image_raw",
        std::bind(&LookForObject::image_callback, this, _1),
        "raw", rmw_qos_profile_sensor_data);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LookForObject::onRunning()
{
    // Get the target color from the parameter
    std::string target_color = node_ptr_->get_parameter("target_color").as_string();

    // Check if the target color exists in the hsv_threshold_dict map
    if (hsv_threshold_dict.find(target_color) == hsv_threshold_dict.end()) {
        std::cerr << "[" << this->name() << "] Target color not found in hsv_threshold_dict!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Get the HSV threshold for the target color
    std::vector<int> hsv_threshold = hsv_threshold_dict.at(target_color);

    if (!received_image_) {
        return BT::NodeStatus::RUNNING;
    }

    // Process the image to look for the object of the target color
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(latest_image_ptr_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv_image,
                cv::Scalar(hsv_threshold[0], hsv_threshold[2], hsv_threshold[4]),
                cv::Scalar(hsv_threshold[1], hsv_threshold[3], hsv_threshold[5]),
                mask);

    std::vector<cv::KeyPoint> keypoints;
    cv::SimpleBlobDetector::Params params;
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(mask, keypoints);

    if (keypoints.size() > 0) {
        std::cout << "[" << this->name() << "] Found object of color " << target_color << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cout << "[" << this->name() << "] Did not find object of color " << target_color << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

void LookForObject::onHalted() {
    image_sub_.shutdown();
    received_image_ = false;
}

void LookForObject::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    latest_image_ptr_ = msg;
    received_image_ = true;
}