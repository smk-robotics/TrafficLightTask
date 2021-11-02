/**
 * @file simple_traffic_light_detector_params.cpp
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Class implementation  for storing all required parameters for simple traffic light detector.
 * @date 2021-11-01
 * @copyright GNU General Public License v3.0.
 */
#include "simple_traffic_light_detector_params.h"

namespace traffic_light_detector {

SimpleTrafficLightDetectorParams::SimpleTrafficLightDetectorParams(const CircleDetectionParams &green_light_params, 
    const CircleDetectionParams &red_light_params, const CircleDetectionParams &yellow_light_params)
    : green_light_detection_params_(green_light_params)
    , red_light_detection_params_(red_light_params)
    , yellow_light_detection_params_(yellow_light_params) {}

CircleDetectionParams SimpleTrafficLightDetectorParams::green_light_params() const noexcept {
    return green_light_detection_params_;
}

CircleDetectionParams SimpleTrafficLightDetectorParams::red_light_params() const noexcept {
    return red_light_detection_params_;
}

CircleDetectionParams SimpleTrafficLightDetectorParams::yellow_light_params() const noexcept {
    return yellow_light_detection_params_;
}

} // namespace traffic_light_detector