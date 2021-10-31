#include "simple_traffic_light_detector.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

int main() {
    cv::Mat origin_image = cv::imread(
        "/home/ksmirnov/local/Software_Patterns_Test/TrafficLightTask/src/traffic_light_fetcher/testdata/3.jpg");
    cv::Mat rgb_image;
    cv::cvtColor(origin_image, rgb_image, cv::COLOR_BGR2HSV);
    cv::imshow("RGB image", rgb_image);
    cv::Mat green_image;
    std::vector<int> lower_green_bound{40, 50, 50};
    std::vector<int> upper_green_bound{90, 255, 255};
    cv::inRange(rgb_image, lower_green_bound, upper_green_bound, green_image);
    cv::imshow("Green image", green_image);
    cv::Mat red_image;
    std::vector<int> lower_red_bound{100, 100, 100};
    std::vector<int> upper_red_bound{180, 255, 255};
    cv::inRange(rgb_image, lower_red_bound, upper_red_bound, red_image);
    cv::imshow("Red image", red_image);

    std::vector<cv::Vec3f> green_circles;
    cv::HoughCircles(green_image, green_circles, cv::HOUGH_GRADIENT, 1, 60, 50, 10, 0, 30);
    std::cout << "[INFO] - Green circles count -> " << green_circles.size() << std::endl;
    for (const auto &c : green_circles) {
        std::cout << "green_circles c[0] -> " << round(c[0]) << std::endl;
        std::cout << "green_circles c[1] -> " << round(c[1]) << std::endl;
        std::cout << "green_circles c[2] -> " << round(c[2]) << std::endl;
        cv::Point2i circle_center(round(c[0]), round(c[1]));
        cv::circle(origin_image, circle_center, c[2], {0, 0, 255}, 2);
    }
    cv::imshow("Origin image", origin_image);
    cv::waitKey();
}