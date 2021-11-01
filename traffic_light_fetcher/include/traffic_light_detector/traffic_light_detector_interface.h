/**
 * @file traffic_light_detector_interface.h
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Interface class for traffic light detectors declaration.
 * @date 2021-11-01
 * @copyright GNU General Public License v3.0.
 */
# pragma once

#include <opencv2/core.hpp>
#include <optional>

namespace traffic_light_detector {
/**
 * @brief Interface class for all traffic light detectors.
 */
class TrafficLightDetectorInterface {
public:
    /**
     * @brief Detects traffic light and calculates it's bounding box size.
     * @param image Input RGB image for traffic light detection.
     * @return std::optional<cv::Vec2i> Full traffic light bounding box size (x, y).
     */
    virtual std::optional<cv::Vec2i> detect_traffic_light(const cv::Mat &image) noexcept = 0;
    /**
     * @brief Destroy the Traffic Light Detector Interface object.
     */
    virtual ~TrafficLightDetectorInterface() = default;
};

} // namespace traffic_light_detector