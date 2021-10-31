# pragma once

#include "traffic_light_detector_interface.h"

namespace traffic_light_detector {

class SimpleTrafficLightDetector : public TrafficLightDetectorInterface {
public:
    SimpleTrafficLightDetector();
    std::optional<cv::Vec2i> detect_traffic_light(const cv::Mat &image) const noexcept override final;

private:
    [[nodiscard]] std::optional<cv::Vec3f> find_biggest_green_traffic_light_() const noexcept;
    [[nodiscard]] std::optional<cv::Vec3f> find_biggest_red_traffic_light_() const noexcept;
    [[nodiscard]] std::optional<cv::Vec3f> find_biggest_yellow_traffic_light_() const noexcept;
    [[nodiscard]] std::optional<cv::Vec2i> calculate_biggest_traffic_light_bbox_(
        std::vector<cv::Vec3f> &traffic_lights) const noexcept;
private: 
    cv::Mat hsv_image_;
};

} // namespace traffic_light_detector