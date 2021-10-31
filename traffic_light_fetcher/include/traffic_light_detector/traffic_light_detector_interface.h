# pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace traffic_light_detector {

class TrafficLightDetectorInterface {
public:
    virtual std::optional<cv::Vec2i> detect_traffic_light(const cv::Mat &image) const noexcept = 0;
    virtual ~TrafficLightDetectorInterface() = default;
};

} // namespace traffic_light_detector