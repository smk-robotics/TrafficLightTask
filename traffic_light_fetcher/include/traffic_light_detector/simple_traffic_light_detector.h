/**
 * @file simple_traffic_light_detector.h
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Simple traffic light detector class declaration.
 * @date 2021-11-01
 * @copyright GNU General Public License v3.0.
 * 
 */
# pragma once

#include "traffic_light_detector_interface.h"
#include "simple_traffic_light_detector_params.h"

namespace traffic_light_detector {
/**
 * @brief Class with simple traffic light detection algorithm based on Hough transform.
 */
class SimpleTrafficLightDetector : public TrafficLightDetectorInterface {
public:
    /**
     * @brief Construct a new Simple Traffic Light Detector object.
     * @param detector_parameters Structure with parameters for simple traffic light detector.
     */
    SimpleTrafficLightDetector(const SimpleTrafficLightDetectorParams &detector_parameters);
    /**
     * @brief Detects nearest (biggest) traffic light and calculates it's bounding box.
     * @param image Input image for traffic light detection.
     * @return std::optional<cv::Vec2i> Traffic light bounding box dimensions (x, y).
     */
    std::optional<cv::Vec2i> detect_traffic_light(const cv::Mat &image) noexcept override final;

private:
    /**
     * @brief Calculates biggest (nearest) full traffic light bounding box size.
     * @param traffic_lights Vector of detected traffic lights. 
     * @return std::optional<cv::Vec2i> Biggest (nearest) traffic light bounding box dimensions (x, y).
     */
    [[nodiscard]] std::optional<cv::Vec2i> calculate_biggest_traffic_light_bbox_(
        std::vector<cv::Vec3f> &traffic_lights) const noexcept;
    /**
     * @brief Find all traffic light circles using given parameters and return biggest (nearest) one.
     * @param detection_params Parameters for specific color (green, red, yellow) traffic light circle detection. 
     * @return std::optional<cv::Vec3f> Biggest (nearest) traffic light circle (x, y, radius).
     */
    [[nodiscard]] std::optional<cv::Vec3f> find_biggest_traffic_light_circle_(
        const CircleDetectionParams &detection_params) const noexcept;

private: 
    /**
     * @brief HSV (hue saturation value) image with H range 0..180 if 8 bit image.
     */
    cv::Mat hsv_image_;
    /**
     * @brief Structure with parameters for traffic light detection algorithms.
     */
    SimpleTrafficLightDetectorParams params_;
};

} // namespace traffic_light_detector