/**
 * @file circle_detection_params.h
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Color specific circle detection parameters structure declaration.
 * @date 2021-11-01
 * @copyright GNU General Public License v3.0.
 */
#pragma once

#include <opencv2/imgproc.hpp>
#include <vector>

namespace traffic_light_detector {
/**
 * @brief Structure for storing color specific circle detection parameters 
 * using in inRange and HoughCircles OpenCV functions.
 */
struct CircleDetectionParams {
    /**
     * @brief Inclusive lower boundary scalar for iamge ranging.
     */
    cv::Scalar color_lower_bound{0, 0, 0};
    /**
     * @brief Inclusive upper boundary scalar for iamge ranging.
     */
     cv::Scalar color_upper_bound{0, 0, 0};
    /**
     * @brief Detection method for OpenCV Hough circle detector. Currently, 
     * the only implemented method is HOUGH_GRADIENT
     */
    int detection_method = cv::HOUGH_GRADIENT;
    /**
     * @brief Inverse ratio of the accumulator resolution to the image 
     * resolution for OpenCV Hough circle detector.
     * @details If dp = 1, the accumulator has the same resolution as the input 
     * image. If dp = 2, the accumulator has half as big width and height.
     */
    double dp = 1.0;
    /**
     * @brief Minimum distance between the centers of the detected circles.
     * @details If the parameter is too small, multiple neighbor circles may be 
     * falsely detected in addition to a true one. If it is too large, some 
     * circles may be missed.
     */
    double min_distance = 0.0;
    /**
     * @brief First method-specific parameter. 
     * @details In case of HOUGH_GRADIENT, it is the higher threshold of the 
     * two passed to the Canny edge detector (the lower one is twice smaller).
     */
    double param_1 = 30.0;
    /**
     * @brief Second method-specific parameter. 
     * @details In case of HOUGH_GRADIENT, it is the accumulator threshold for 
     * the circle centers at the detection stage. The smaller it is, the more 
     * false circles may be detected. Circles, corresponding to the larger 
     * accumulator values, will be returned first.
     */
    double param_2 = 50.0;
    /**
     * @brief Minimum circle radius [pixels].
     */
    int min_radius = 0;
    /**
     * @brief Maximum circle radius. 
     * @details If <= 0, uses the maximum image dimension. If < 0, returns 
     * centers without finding the radius.
     */
    int max_radius = 0;
};

} // namespace traffic_light_detector