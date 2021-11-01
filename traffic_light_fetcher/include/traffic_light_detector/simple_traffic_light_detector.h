# pragma once

#include "traffic_light_detector_interface.h"
#include "simple_traffic_light_detector_params.h"

namespace traffic_light_detector {

class SimpleTrafficLightDetector : public TrafficLightDetectorInterface {
public:
    SimpleTrafficLightDetector(const SimpleTrafficLightDetectorParams &detector_parameters);
    std::optional<cv::Vec2i> detect_traffic_light(const cv::Mat &image) noexcept override final;
private:
    [[nodiscard]] std::optional<cv::Vec2i> calculate_biggest_traffic_light_bbox_(
        std::vector<cv::Vec3f> &traffic_lights) const noexcept;
    [[nodiscard]] std::optional<cv::Vec3f> find_biggest_traffic_light_circle_(
        const CircleDetectionParams &detection_params) const noexcept;
private: 
    cv::Mat hsv_image_;
    SimpleTrafficLightDetectorParams params_;
};

} // namespace traffic_light_detector