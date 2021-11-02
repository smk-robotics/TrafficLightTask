/**
 * @file main.cpp
 * @author Kirill Smirnov <smk.robotics@gmail.com>
 * @brief Main file for running ROS traffic light analysis node.
 * @date 2021-11-02
 * @copyright GNU General Public License v3.0.
 */
#include "traffic_light_analysis_node.h"
#include <iostream>

using namespace traffic_light_analysis;

int main(int argc, char** argv)
{
  try {
    ros::init(argc, argv, "traffic_light_analysis");
    ros::NodeHandlePtr pnh(new ros::NodeHandle("~"));
    TrafficLightAnalysisNode traffic_light_analysis_node(pnh);
    ros::spin();
  } catch (ros::Exception& e) {  // Handle ROS exceptions.
    std::cerr << "[ERROR]:[traffic_light_analysis] - " << e.what() << std::endl;
  } catch (std::exception& e) {  // Handle std errors.
    std::cerr << "[ERROR]:[traffic_light_analysis] - " << e.what() << std::endl;
  }
  return EXIT_SUCCESS;
}
