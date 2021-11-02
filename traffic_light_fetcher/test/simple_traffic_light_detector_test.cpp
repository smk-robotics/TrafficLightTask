#include "simple_traffic_light_detector.h"
#include <gtest/gtest.h>
#include <opencv2/highgui.hpp>

using namespace traffic_light_detector;

class SimpleTrafficLightDetectorTest : public testing::Test {
protected:
    void SetUp() {
        const std::string current_file_path = __FILE__;
        const std::string src_directory_path_ = current_file_path.substr(0, current_file_path.rfind("/test"));
        testdata_directory_ = src_directory_path_ + "/testdata";

        green_light_detection_params_.color_lower_bound = {40, 50, 50};
        green_light_detection_params_.color_upper_bound = {90, 255, 255};
        green_light_detection_params_.min_distance = 60.0;
        green_light_detection_params_.param_1 = 50.0;
        green_light_detection_params_.param_2 = 10.0;
        green_light_detection_params_.max_radius = 30.0;

        red_light_detection_params_.color_lower_bound = {160, 100, 100};
        red_light_detection_params_.color_upper_bound = {180, 255, 255};
        red_light_detection_params_.min_distance = 60.0;
        red_light_detection_params_.param_1 = 50.0;
        red_light_detection_params_.param_2 = 10.0;
        red_light_detection_params_.max_radius = 30.0;

        yellow_light_detection_params_.color_lower_bound = {15, 150, 150};
        yellow_light_detection_params_.color_upper_bound = {35, 255, 255};
        yellow_light_detection_params_.min_distance = 60.0;
        yellow_light_detection_params_.param_1 = 50.0;
        yellow_light_detection_params_.param_2 = 10.0;
        yellow_light_detection_params_.max_radius = 30.0;

        simplet_traffic_light_detector_params_ = std::make_shared<SimpleTrafficLightDetectorParams>(
            green_light_detection_params_, red_light_detection_params_, yellow_light_detection_params_);
    }

protected:
    CircleDetectionParams green_light_detection_params_;
    CircleDetectionParams red_light_detection_params_;
    CircleDetectionParams yellow_light_detection_params_;
    std::shared_ptr<SimpleTrafficLightDetectorParams> simplet_traffic_light_detector_params_;
    std::string testdata_directory_;
};

TEST_F(SimpleTrafficLightDetectorTest, simple_traffic_light_detector_first_test) {
    const std::string img_file = testdata_directory_ + "/1.jpg";
    cv::Mat origin_image = cv::imread(img_file);

    std::shared_ptr<TrafficLightDetectorInterface> traffic_light_detector;
    ASSERT_NO_THROW(traffic_light_detector = std::make_shared<SimpleTrafficLightDetector>(
        *simplet_traffic_light_detector_params_));
    auto traffic_light = traffic_light_detector->detect_traffic_light(origin_image);
    EXPECT_TRUE(traffic_light.has_value());
    EXPECT_EQ(traffic_light.value()[0], 11);
    EXPECT_EQ(traffic_light.value()[1], 33);
}

TEST_F(SimpleTrafficLightDetectorTest, simple_traffic_light_detector_second_test) {
    const std::string img_file = testdata_directory_ + "/2.jpg";
    cv::Mat origin_image = cv::imread(img_file);

    std::shared_ptr<TrafficLightDetectorInterface> traffic_light_detector;
    ASSERT_NO_THROW(traffic_light_detector = std::make_shared<SimpleTrafficLightDetector>(
        *simplet_traffic_light_detector_params_));
    auto traffic_light = traffic_light_detector->detect_traffic_light(origin_image);
    EXPECT_TRUE(traffic_light.has_value());
    EXPECT_EQ(traffic_light.value()[0], 10);
    EXPECT_EQ(traffic_light.value()[1], 30);
}

TEST_F(SimpleTrafficLightDetectorTest, simple_traffic_light_detector_third_test) {
    const std::string img_file = testdata_directory_ + "/3.jpg";
    cv::Mat origin_image = cv::imread(img_file);

    std::shared_ptr<TrafficLightDetectorInterface> traffic_light_detector;
    ASSERT_NO_THROW(traffic_light_detector = std::make_shared<SimpleTrafficLightDetector>(
        *simplet_traffic_light_detector_params_));
    auto traffic_light = traffic_light_detector->detect_traffic_light(origin_image);
    EXPECT_TRUE(traffic_light.has_value());
    EXPECT_EQ(traffic_light.value()[0], 9);
    EXPECT_EQ(traffic_light.value()[1], 27);
}