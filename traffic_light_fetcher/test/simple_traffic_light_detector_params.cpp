#include "simple_traffic_light_detector_params.h"
#include <gtest/gtest.h>

using namespace traffic_light_detector;

TEST(SimpleTrafficLightDetectorParamsTest, default_circle_detection_params_test) {
    CircleDetectionParams default_detection_params;
    ASSERT_EQ(default_detection_params.color_lower_bound[0], 0);
    ASSERT_EQ(default_detection_params.color_lower_bound[1], 0);
    ASSERT_EQ(default_detection_params.color_lower_bound[2], 0);
    ASSERT_EQ(default_detection_params.color_upper_bound[0], 0);
    ASSERT_EQ(default_detection_params.color_upper_bound[1], 0);
    ASSERT_EQ(default_detection_params.color_upper_bound[2], 0);
    ASSERT_EQ(default_detection_params.detection_method, cv::HOUGH_GRADIENT);
    ASSERT_DOUBLE_EQ(default_detection_params.dp, 1.0);
    ASSERT_DOUBLE_EQ(default_detection_params.min_distance, 0.0);
    ASSERT_DOUBLE_EQ(default_detection_params.param_1, 30.0);
    ASSERT_DOUBLE_EQ(default_detection_params.param_2, 50.0);
    ASSERT_EQ(default_detection_params.min_radius, 0);
    ASSERT_EQ(default_detection_params.max_radius, 0);
}

TEST(SimpleTrafficLightDetectorParamsTest, circle_detection_params_setup_test) {
    CircleDetectionParams green_light_detection_params;
    green_light_detection_params.color_lower_bound = {40, 50, 50};
    ASSERT_EQ(green_light_detection_params.color_lower_bound[0], 40);
    ASSERT_EQ(green_light_detection_params.color_lower_bound[1], 50);
    ASSERT_EQ(green_light_detection_params.color_lower_bound[2], 50);
    green_light_detection_params.color_upper_bound = {90, 255, 255};
    ASSERT_EQ(green_light_detection_params.color_upper_bound[0], 90);
    ASSERT_EQ(green_light_detection_params.color_upper_bound[1], 255);
    ASSERT_EQ(green_light_detection_params.color_upper_bound[2], 255);
    ASSERT_EQ(green_light_detection_params.detection_method, 3);
    green_light_detection_params.min_distance = 60.0;
    ASSERT_DOUBLE_EQ(green_light_detection_params.min_distance, 60.0);
    green_light_detection_params.param_1 = 50.0;
    ASSERT_DOUBLE_EQ(green_light_detection_params.param_1, 50.0);
    green_light_detection_params.param_2 = 10.0;
    ASSERT_DOUBLE_EQ(green_light_detection_params.param_2, 10.0);
    green_light_detection_params.max_radius = 30.0;
    ASSERT_DOUBLE_EQ(green_light_detection_params.max_radius, 30.0);
}

TEST(SimpleTrafficLightDetectorParamsTest, simple_traffic_light_detector_params_test) {
    CircleDetectionParams green_light_detection_params;
    green_light_detection_params.color_lower_bound = {40, 50, 50};
    green_light_detection_params.color_upper_bound = {90, 255, 255};
    green_light_detection_params.min_distance = 60.0;
    green_light_detection_params.param_1 = 50.0;
    green_light_detection_params.param_2 = 10.0;
    green_light_detection_params.max_radius = 30.0;
    
    CircleDetectionParams red_light_detection_params;
    red_light_detection_params.color_lower_bound = {100, 100, 100};
    red_light_detection_params.color_upper_bound = {180, 255, 255};
    red_light_detection_params.min_distance = 60.0;
    red_light_detection_params.param_1 = 50.0;
    red_light_detection_params.param_2 = 10.0;
    red_light_detection_params.max_radius = 30.0;

    CircleDetectionParams yellow_light_detection_params;
    yellow_light_detection_params.color_lower_bound = {15, 150, 150};
    yellow_light_detection_params.color_upper_bound = {35, 255, 255};
    yellow_light_detection_params.min_distance = 60.0;
    yellow_light_detection_params.param_1 = 50.0;
    yellow_light_detection_params.param_2 = 10.0;
    yellow_light_detection_params.max_radius = 30.0;
    
    SimpleTrafficLightDetectorParams simple_traffic_light_detection_params(
        green_light_detection_params, red_light_detection_params, yellow_light_detection_params);
    
    ASSERT_EQ(simple_traffic_light_detection_params.green_light_params().color_lower_bound[0], 40);
    ASSERT_EQ(simple_traffic_light_detection_params.green_light_params().color_lower_bound[1], 50);
    ASSERT_EQ(simple_traffic_light_detection_params.green_light_params().color_lower_bound[1], 50);
    ASSERT_EQ(simple_traffic_light_detection_params.green_light_params().color_upper_bound[0], 90);
    ASSERT_EQ(simple_traffic_light_detection_params.green_light_params().color_upper_bound[1], 255);
    ASSERT_EQ(simple_traffic_light_detection_params.green_light_params().color_upper_bound[1], 255);
    
    ASSERT_EQ(simple_traffic_light_detection_params.red_light_params().color_lower_bound[0], 100);
    ASSERT_EQ(simple_traffic_light_detection_params.red_light_params().color_lower_bound[1], 100);
    ASSERT_EQ(simple_traffic_light_detection_params.red_light_params().color_lower_bound[1], 100);
    ASSERT_EQ(simple_traffic_light_detection_params.red_light_params().color_upper_bound[0], 180);
    ASSERT_EQ(simple_traffic_light_detection_params.red_light_params().color_upper_bound[1], 255);
    ASSERT_EQ(simple_traffic_light_detection_params.red_light_params().color_upper_bound[1], 255);

    ASSERT_EQ(simple_traffic_light_detection_params.yellow_light_params().color_lower_bound[0], 15);
    ASSERT_EQ(simple_traffic_light_detection_params.yellow_light_params().color_lower_bound[1], 150);
    ASSERT_EQ(simple_traffic_light_detection_params.yellow_light_params().color_lower_bound[1], 150);
    ASSERT_EQ(simple_traffic_light_detection_params.yellow_light_params().color_upper_bound[0], 35);
    ASSERT_EQ(simple_traffic_light_detection_params.yellow_light_params().color_upper_bound[1], 255);
    ASSERT_EQ(simple_traffic_light_detection_params.yellow_light_params().color_upper_bound[1], 255);
}