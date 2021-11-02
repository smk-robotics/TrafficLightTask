/**
 * @file traffic_light_fetcher_node.cpp
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Traffic light fetcher node implementation.
 * @date 2021-11-02
 * @copyright GNU General Public License v3.0.
 */
#include "traffic_light_fetcher_node.h"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>

namespace traffic_light_fetcher {

TrafficLightFetcherNode::TrafficLightFetcherNode([[maybe_unused]] const ros::NodeHandlePtr& nodehandler) 
    : pnh_(nodehandler) {
    setup_node_communications_();
    setup_traffic_light_detector_params_();
    traffic_light_detector_ = std::make_shared<SimpleTrafficLightDetector>(*traffic_light_detector_params_);
}

void TrafficLightFetcherNode::setup_node_communications_() noexcept {
    input_image_sub_ = pnh_->subscribe("input_image", 1, &TrafficLightFetcherNode::input_image_callback_, this);
    traffic_light_detected_signal_pub_ = pnh_->advertise<std_msgs::Bool>("traffic_light_detected", 1, true);
    traffic_light_size_pub_ = pnh_->advertise<geometry_msgs::Vector3>("traffic_light_size", 1, true);
}

void TrafficLightFetcherNode::input_image_callback_(const sensor_msgs::ImageConstPtr &img) const noexcept{
    const auto cv_bridge_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    const auto detected_traffic_light_bbox = traffic_light_detector_->detect_traffic_light(cv_bridge_img_ptr->image);
    if (detected_traffic_light_bbox.has_value()) {
        publish_detected_traffic_light_(detected_traffic_light_bbox.value());
    }
}

void TrafficLightFetcherNode::publish_detected_traffic_light_(const cv::Vec2i &traffic_light_size) const noexcept {
    std_msgs::Bool traffic_light_detector_signal;
    traffic_light_detector_signal.data = true;
    
    geometry_msgs::Vector3 traffic_light_size_msg;
    traffic_light_size_msg.x = traffic_light_size[0];
    traffic_light_size_msg.y = traffic_light_size[1];
    traffic_light_size_msg.z = 0.0;
    
    traffic_light_size_pub_.publish(traffic_light_size_msg);
    traffic_light_detected_signal_pub_.publish(traffic_light_detector_signal);
}

void TrafficLightFetcherNode::setup_traffic_light_detector_params_() noexcept {
    CircleDetectionParams green_light_detection_params;
    const auto green_color_lower_bound = pnh_->param<std::vector<int>>(
        "traffic_light_detector/green_light/color_lower_bound", {0, 0, 0});
    green_light_detection_params.color_lower_bound[0] = green_color_lower_bound[0];
    green_light_detection_params.color_lower_bound[1] = green_color_lower_bound[1];
    green_light_detection_params.color_lower_bound[2] = green_color_lower_bound[2];
    const auto green_color_upper_bound = pnh_->param<std::vector<int>>(
        "traffic_light_detector/green_light/color_upper_bound", {0, 0, 0});
    green_light_detection_params.color_upper_bound[0] = green_color_upper_bound[0];
    green_light_detection_params.color_upper_bound[1] = green_color_upper_bound[1];
    green_light_detection_params.color_upper_bound[2] = green_color_upper_bound[2];
    green_light_detection_params.detection_method =  pnh_->param("traffic_light_detector/green_light/detection_method", 3);
    green_light_detection_params.dp = pnh_->param("traffic_light_detector/green_light/inverse_ratio", 1.0);
    green_light_detection_params.min_distance = pnh_->param("traffic_light_detector/green_light/min_distance", 60.0);
    green_light_detection_params.param_1 = pnh_->param("traffic_light_detector/green_light/higher_threshold", 50.0);
    green_light_detection_params.param_2 = pnh_->param("traffic_light_detector/green_light/accumulator_threshold", 10.0);
    green_light_detection_params.min_radius = pnh_->param("traffic_light_detector/green_light/min_radius", 0.0);
    green_light_detection_params.max_radius = pnh_->param("traffic_light_detector/green_light/max_radius", 30.0);

    CircleDetectionParams red_light_detection_params;
    const auto red_color_lower_bound = pnh_->param<std::vector<int>>(
        "traffic_light_detector/red_light/color_lower_bound", {0, 0, 0});
    red_light_detection_params.color_lower_bound[0] = red_color_lower_bound[0];
    red_light_detection_params.color_lower_bound[1] = red_color_lower_bound[1];
    red_light_detection_params.color_lower_bound[2] = red_color_lower_bound[2];
    const auto red_color_upper_bound = pnh_->param<std::vector<int>>(
        "traffic_light_detector/red_light/color_upper_bound", {0, 0, 0});
    red_light_detection_params.color_upper_bound[0] = red_color_upper_bound[0];
    red_light_detection_params.color_upper_bound[1] = red_color_upper_bound[1];
    red_light_detection_params.color_upper_bound[2] = red_color_upper_bound[2];
    red_light_detection_params.detection_method =  pnh_->param("traffic_light_detector/red_light/detection_method", 3);
    red_light_detection_params.dp = pnh_->param("traffic_light_detector/red_light/inverse_ratio", 1.0);
    red_light_detection_params.min_distance = pnh_->param("traffic_light_detector/red_light/min_distance", 60.0);
    red_light_detection_params.param_1 = pnh_->param("traffic_light_detector/red_light/higher_threshold", 50.0);
    red_light_detection_params.param_2 = pnh_->param("traffic_light_detector/red_light/accumulator_threshold", 10.0);
    red_light_detection_params.min_radius = pnh_->param("traffic_light_detector/red_light/min_radius", 0.0);
    red_light_detection_params.max_radius = pnh_->param("traffic_light_detector/red_light/max_radius", 30.0);

    CircleDetectionParams yellow_light_detection_params;
    const auto yellow_color_lower_bound = pnh_->param<std::vector<int>>(
        "traffic_light_detector/yellow_light/color_lower_bound", {0, 0, 0});
    yellow_light_detection_params.color_lower_bound[0] = yellow_color_lower_bound[0];
    yellow_light_detection_params.color_lower_bound[1] = yellow_color_lower_bound[1];
    yellow_light_detection_params.color_lower_bound[2] = yellow_color_lower_bound[2];
    const auto yellow_color_upper_bound = pnh_->param<std::vector<int>>(
        "traffic_light_detector/yellow_light/color_upper_bound", {0, 0, 0});
    yellow_light_detection_params.color_upper_bound[0] = yellow_color_upper_bound[0];
    yellow_light_detection_params.color_upper_bound[1] = yellow_color_upper_bound[1];
    yellow_light_detection_params.color_upper_bound[2] = yellow_color_upper_bound[2];
    yellow_light_detection_params.detection_method =  pnh_->param("traffic_light_detector/yellow_light/detection_method", 3);
    yellow_light_detection_params.dp = pnh_->param("traffic_light_detector/yellow_light/inverse_ratio", 1.0);
    yellow_light_detection_params.min_distance = pnh_->param("traffic_light_detector/yellow_light/min_distance", 60.0);
    yellow_light_detection_params.param_1 = pnh_->param("traffic_light_detector/yellow_light/higher_threshold", 50.0);
    yellow_light_detection_params.param_2 = pnh_->param("traffic_light_detector/yellow_light/accumulator_threshold", 10.0);
    yellow_light_detection_params.min_radius = pnh_->param("traffic_light_detector/yellow_light/min_radius", 0.0);
    yellow_light_detection_params.max_radius = pnh_->param("traffic_light_detector/yellow_light/max_radius", 30.0);

    traffic_light_detector_params_ = std::make_shared<SimpleTrafficLightDetectorParams>(green_light_detection_params,
        red_light_detection_params, yellow_light_detection_params);
}

} // namespace traffic_light_fetcher