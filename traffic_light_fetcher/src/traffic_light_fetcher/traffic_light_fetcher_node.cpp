#include "traffic_light_fetcher_node.h"

namespace traffic_light_fetcher {

TrafficLightFetcherNode::TrafficLightFetcherNode([[maybe_unused]] const ros::NodeHandlePtr& pnh) {

    CircleDetectionParams green_light_detection_params;
    const auto green_color_lower_bound = pnh->param<std::vector<int>>(
        "traffic_light_detector/green_light/color_lower_bound", {0, 0, 0});
    green_light_detection_params.color_lower_bound[0] = green_color_lower_bound[0];
    green_light_detection_params.color_lower_bound[1] = green_color_lower_bound[1];
    green_light_detection_params.color_lower_bound[2] = green_color_lower_bound[2];
    const auto green_color_upper_bound = pnh->param<std::vector<int>>(
        "traffic_light_detector/green_light/color_upper_bound", {0, 0, 0});
    green_light_detection_params.color_upper_bound[0] = green_color_upper_bound[0];
    green_light_detection_params.color_upper_bound[1] = green_color_upper_bound[1];
    green_light_detection_params.color_upper_bound[2] = green_color_upper_bound[2];
}

} // namespace traffic_light_fetcher