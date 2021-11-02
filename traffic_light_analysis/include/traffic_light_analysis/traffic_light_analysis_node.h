/**
 * @file traffic_light_analysis_node.h
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Traffic light analysis node declaration.
 * @date 2021-11-02
 * @copyright GNU General Public License v3.0.
 */
#pragma once

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <ros/node_handle.h>

namespace traffic_light_analysis {

class TrafficLightAnalysisNode {
public:
    /**
     * @brief Construct a new Traffic Light Analysis Node object.
     * @param nodehandler Private ROS nodehandler.
     */
    TrafficLightAnalysisNode(const ros::NodeHandlePtr& nodehandler);

private:
    /**
     * @brief Sets up ROS publishers and subscribers for node communication. 
     */
    void setup_node_communications_() noexcept;
    /**
     * @brief Callback for traffic light detection signal.
     * @param signal Signal that traffic light was detected.
     */
    void traffic_light_detected_signal_callback_(const std_msgs::Bool &signal) const noexcept;
    /**
     * @brief Callback for detected traffic light bounding box size.
     * @param traffic_light_size Detected traffic light bounding box size.
     */
    void traffic_light_size_callback_(const geometry_msgs::Vector3 &traffic_light_size) noexcept;
    
private:
    /**
     * @brief ROS nodehandle for setting up node communication and gettin parameters
     * from ROS parameter server.
     */
    ros::NodeHandlePtr pnh_;
    /**
     * @brief ROS subscriber for traffic light detection signal.
     */
    ros::Subscriber traffic_light_detected_signal_sub_;
    /**
     * @brief ROS subscriber for detected traffic light bounding box size.
     */
    ros::Subscriber traffic_light_size_sub_;
    /**
     * @brief ROS publisher for calculated zone height.
     */
    ros::Publisher zone_height_pub_;
    /**
     * @brief Last detected traffic light bounding box size.
     */
    geometry_msgs::Vector3 last_detected_traffic_light_bbox_size_;
};

} // namespace traffic_light_analysis