#include "simple_traffic_light_detector.h"
#include <opencv2/imgproc.hpp>

namespace traffic_light_detector {

SimpleTrafficLightDetector::SimpleTrafficLightDetector(const SimpleTrafficLightDetectorParams &detector_parameters)
	: params_(detector_parameters) {}

std::optional<cv::Vec2i> SimpleTrafficLightDetector::detect_traffic_light(const cv::Mat &origin_image) noexcept {
	cv::cvtColor(origin_image, hsv_image_, cv::COLOR_BGR2HSV);

  	const auto green_light  = find_biggest_traffic_light_circle_(params_.green_light_params());
  	const auto red_light    = find_biggest_traffic_light_circle_(params_.red_light_params());
  	const auto yellow_light = find_biggest_traffic_light_circle_(params_.yellow_light_params());

  	std::vector<cv::Vec3f> detected_traffic_lights;
  	if (green_light.has_value()) {
		detected_traffic_lights.emplace_back(green_light.value());
  	}
  	if (red_light.has_value()) {
  		detected_traffic_lights.emplace_back(red_light.value());
  	}
  	if (yellow_light.has_value()) {
  		detected_traffic_lights.emplace_back(yellow_light.value());
  	}

  	const auto traffic_light_bbox = calculate_biggest_traffic_light_bbox_(detected_traffic_lights);
  	return traffic_light_bbox;
}

struct GreaterCircleSizeComparasion {
  	bool operator()(cv::Vec3f a, cv::Vec3f b) const { return a[2] > b[2]; }
};

std::optional<cv::Vec3f> SimpleTrafficLightDetector::find_biggest_traffic_light_circle_(
	const CircleDetectionParams &detection_params) const noexcept {
  	cv::Mat mono_color_img;
  	cv::inRange(hsv_image_, detection_params.color_lower_bound, detection_params.color_upper_bound, mono_color_img);
	std::vector<cv::Vec3f> detected_circles;
  	cv::HoughCircles(mono_color_img, detected_circles, 
	   				 detection_params.detection_method, 
	  				 detection_params.dp, 
					 detection_params.min_distance, 
					 detection_params.param_1, 
					 detection_params.param_2, 
					 detection_params.min_radius, 
					 detection_params.max_radius);
	std::optional<cv::Vec3f> result_detected_circle;
	if (detected_circles.size() > 0) {
  		std::sort(detected_circles.begin(), detected_circles.end(), GreaterCircleSizeComparasion());
		result_detected_circle = *detected_circles.begin();
	}
  	return result_detected_circle;
}

std::optional<cv::Vec2i> SimpleTrafficLightDetector::calculate_biggest_traffic_light_bbox_(
	std::vector<cv::Vec3f> &traffic_lights) const noexcept {
	cv::Vec2i biggest_traffic_light_bbox;
	std::sort(traffic_lights.begin(), traffic_lights.end(), GreaterCircleSizeComparasion());
	biggest_traffic_light_bbox[0] = round(traffic_lights[0][2]);
	biggest_traffic_light_bbox[1] = round(traffic_lights[0][2]) * 3;
	return biggest_traffic_light_bbox;
}

} // namespace traffic_light_detector