/**
 * @file simple_traffic_light_detector_params.h
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Class declaration for storing all required parameters for simple traffic light detector.
 * @date 2021-11-01
 * @copyright GNU General Public License v3.0.
 */
#pragma once

#include "circle_detection_params.h"

namespace traffic_light_detector {
/**
 * @brief Class for storing all parameters for simple traffic light detector.
 */
class SimpleTrafficLightDetectorParams{ 
public:
    /**
     * @brief Construct a new Simple Traffic Light Detector Params object
     * @param green_light_params Parameters for green light circle detection.
     * @param red_light_params Parameters for red light circle detection.
     * @param yellow_light_params Parameters for yellow light circle detection.
     */
    SimpleTrafficLightDetectorParams(const CircleDetectionParams &green_light_params, 
                                     const CircleDetectionParams &red_light_params, 
                                     const CircleDetectionParams &yellow_light_params);
    /**
     * @brief Getter for green traffic light circle detection parameters.
     * @return CircleDetectionParams specific for green light circle.
     */
    CircleDetectionParams green_light_params() const noexcept;
    /**
     * @brief Getter for red traffic light circle detection parameters.
     * @return CircleDetectionParams specific for red light circle.
     */
    CircleDetectionParams red_light_params() const noexcept;
    /**
     * @brief Getter for yellow traffic light circle detection parameters.
     * @return CircleDetectionParams specific for yellow light circle.
     */
    CircleDetectionParams yellow_light_params() const noexcept;
private:
    /**
     * @brief Parameters for green traffic light circle detection.
     */
    CircleDetectionParams green_light_detection_params_;
    /**
     * @brief Parameters for red traffic light circle detection.
     */
    CircleDetectionParams red_light_detection_params_;
    /**
     * @brief Parameters for yellow traffic light circle detection.
     */
    CircleDetectionParams yellow_light_detection_params_;
};

} // namespace traffic_light_detector