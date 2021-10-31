#include <opencv2/imgproc.hpp>
#include "simple_traffic_light_detector.h"

namespace traffic_light_detector {

SimpleTrafficLightDetector::SimpleTrafficLightDetector() {
	[[maybe_unused]] volatile int i = 0;
}

std::optional<cv::Vec2i> SimpleTrafficLightDetector::detect_traffic_light(const cv::Mat &origin_image) const noexcept {
  	cv::cvtColor(origin_image, hsv_image_, cv::COLOR_BGR2HSV);
  	std::vector<cv::Vec3f> traffic_lights;
  	const auto green_light  = find_biggest_green_traffic_light_();
  	const auto red_light    = find_biggest_red_traffic_light_();
  	const auto yellow_light = find_biggest_yellow_traffic_light_();
  	if (green_light.has_value()) {
		traffic_lights.emplace_back(green_light.value());
  	}
  	if (red_light.has_value()) {
  		traffic_lights.emplace_back(red_light.value());
  	}
  	if (yellow_light.has_value()) {
  		traffic_lights.emplace_back(yellow_light.value());
  	}
  	const auto traffic_light_bbox = calculate_biggest_traffic_light_bbox_(traffic_lights);
  	return traffic_light_bbox;
}

struct CircleLightLess 
{
  	bool operator()(cv::Vec3f a, cv::Vec3f b) const { return a[2] < b[2]; }
};

std::optional<cv::Vec3f> SimpleTrafficLightDetector::find_biggest_green_traffic_light_() const noexcept 
{
  	const std::vector<int> lower_green_bound{40, 50, 50};
  	const std::vector<int> upper_green_bound{90, 255, 255};
  	cv::Mat green_image;
  	cv::inRange(hsv_image_, lower_green_bound, upper_green_bound, green_image);
  	std::vector<cv::Vec3f> green_circles;
  	cv::HoughCircles(green_image, green_circles, cv::HOUGH_GRADIENT, 1, 60, 50, 10, 0, 30);
  	std::sort(green_circles.begin(), green_circles.end(), CircleLightLess());
  	return std::make_optional<cv::Vec3f>(*green_circles.begin());
}

std::optional<cv::Vec3f> SimpleTrafficLightDetector::find_biggest_red_traffic_light_() const noexcept 
{
  	const std::vector<int> lower_red_bound{100, 100, 100};
  	const std::vector<int> upper_red_bound{180, 255, 255};
  	cv::Mat red_image;
  	cv::inRange(hsv_image_, lower_red_bound, upper_red_bound, red_image);
  	std::vector<cv::Vec3f> red_circles;
  	cv::HoughCircles(red_image, red_circles, cv::HOUGH_GRADIENT, 1, 80, 50, 10, 0, 30);
  	std::sort(red_circles.begin(), red_circles.end(), CircleLightLess());
  	return std::make_optional<cv::Vec3f>(*red_circles.begin());
}

std::optional<cv::Vec3f> SimpleTrafficLightDetector::find_biggest_yellow_traffic_light_() const noexcept 
{
  	const std::vector<int> lower_yellow_bound{15, 150, 150};
  	const std::vector<int> upper_yellow_bound{35, 255, 255};
  	cv::Mat yellow_image;
  	cv::inRange(hsv_image_, lower_yellow_bound, upper_yellow_bound, yellow_image);
  	std::vector<cv::Vec3f> yellow_circles;
  	cv::HoughCircles(yellow_image, yellow_circles, cv::HOUGH_GRADIENT, 1, 30, 50, 10, 0, 30);
  	std::sort(yellow_circles.begin(), yellow_circles.end(), CircleLightLess());
  	return std::make_optional<cv::Vec3f>(*yellow_circles.begin());
}

std::optional<cv::Vec2i> SimpleTrafficLightDetector::calculate_biggest_traffic_light_bbox_(
	std::vector<cv::Vec3f> &traffic_lights) const noexcept 
{
	cv::Vec2i biggest_traffic_light_bbox;
	std::sort(traffic_lights.begin(), traffic_lights.end(), CircleLightLess());
	biggest_traffic_light_bbox[0] = round(traffic_lights[0][0]);
	biggest_traffic_light_bbox[1] = round(traffic_lights[0][0]) * 3;
	return biggest_traffic_light_bbox;
}

} // namespace traffic_light_detector