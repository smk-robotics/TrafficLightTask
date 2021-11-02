/**
 * @file traffic_light_fetcher_node.h
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Traffic light fetcher node declaration.
 * @date 2021-11-02
 * @copyright GNU General Public License v3.0.
 */
#pragma once

#include "simple_traffic_light_detector.h"
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>

using namespace traffic_light_detector;

namespace traffic_light_fetcher {
/**
 * @brief Class with ROS node for traffic light fetching.
 */
class TrafficLightFetcherNode {
public:
    /**
     * @brief Construct a new Traffic Light Fetcher Node object
     * @param nodehandler Private ROS nodehandler.
     */
    TrafficLightFetcherNode(const ros::NodeHandlePtr& nodehandler);
private:
    /**
     * @brief Get all parameters for traffic light detector from ROS server add
     * stores it in traffic light detector parameters class property.
     */
    void setup_traffic_light_detector_params_() noexcept;
    /**
     * @brief Sets up ROS publishers and subscribers for node communication. 
     */
    void setup_node_communications_() noexcept;
    /**
     * @brief Callback function for input image with possible traffic light.
     * @param img Image that will passed to traffic light detector.
     */
    void input_image_callback_(const sensor_msgs::ImageConstPtr &img) const noexcept;
    /**
     * @brief Publish all data for detected traffic light.
     * @details Publish signal that traffic light was detected and it's bounding box size.
     * @param traffic_light_size Detected traffic light bounding box size.
     */
    void publish_detected_traffic_light_(const cv::Vec2i &traffic_light_size) const noexcept;
private:
    /**
     * @brief ROS nodehandle for setting up node communication and gettin parameters
     * from ROS parameter server.
     */
    ros::NodeHandlePtr pnh_;
    /**
     * @brief ROS subscriber for input image for traffic light fetching.
     */
    ros::Subscriber input_image_sub_;
    /**
     * @brief ROS publisher for traffic light detection signal.
     */
    ros::Publisher traffic_light_detected_signal_pub_;
    /**
     * @brief ROS publisher for detected traffic light bounding box size.
     * 
     */
    ros::Publisher traffic_light_size_pub_;
    /**
     * @brief Custom structure with parameters for traffic light detector.
     * 
     */
    std::shared_ptr<SimpleTrafficLightDetectorParams> traffic_light_detector_params_;
    /**
     * @brief Interface for traffic light detector.
     */
    std::shared_ptr<TrafficLightDetectorInterface> traffic_light_detector_;
};

} // namespace traffic_light_fetcher