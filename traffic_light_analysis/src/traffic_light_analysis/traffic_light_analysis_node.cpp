/**
 * @file traffic_light_analysis_node.cpp
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Traffic light analysis node implementation.
 * @date 2021-11-02
 * @copyright GNU General Public License v3.0.
 */
#include "traffic_light_analysis_node.h"
#include <std_msgs/Float32.h>

namespace traffic_light_analysis {

TrafficLightAnalysisNode::TrafficLightAnalysisNode(const ros::NodeHandlePtr& nodehandler)
    : pnh_(nodehandler) {
    setup_node_communications_();
}

void TrafficLightAnalysisNode::setup_node_communications_() noexcept {
    traffic_light_detected_signal_sub_ = pnh_->subscribe(
        "traffic_light_detected", 1, &TrafficLightAnalysisNode::traffic_light_detected_signal_callback_, this);
    traffic_light_size_sub_ = pnh_->subscribe(
        "traffic_light_size", 1, &TrafficLightAnalysisNode::traffic_light_size_callback_, this);
    zone_height_pub_ = pnh_->advertise<std_msgs::Float32>("zone_height", 1, true);
}

void TrafficLightAnalysisNode::traffic_light_detected_signal_callback_(const std_msgs::Bool &signal) const noexcept {
    if (signal.data == true) {
        std_msgs::Float32 zone_height;
        zone_height.data = last_detected_traffic_light_bbox_size_.y / 3.0f;
        zone_height_pub_.publish(zone_height);
    }
}

void TrafficLightAnalysisNode::traffic_light_size_callback_(const geometry_msgs::Vector3 &traffic_light_size) noexcept {
    last_detected_traffic_light_bbox_size_ = traffic_light_size;
}

} // namespace traffic_light_analysis