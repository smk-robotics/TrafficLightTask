#pragma once

#include "simple_traffic_light_detector.h"
#include <ros/node_handle.h>

using namespace traffic_light_detector;

namespace traffic_light_fetcher {

class TrafficLightFetcherNode {
public:
    TrafficLightFetcherNode(const ros::NodeHandlePtr& nodehandler);

private:
    // SimpleTrafficLightDetectorParams traffic_light_detector_params_;
    // std::shared_ptr<TrafficLightDetectorInterface> traffic_light_detector_;
};

} // namespace traffic_light_fetcher