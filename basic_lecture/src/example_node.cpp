/**
 * @file example_node.cpp
 * @brief Load parameter, then subcribe and publish a topic, and provide the service.
 */

#include <ros/ros.h>
#include "basic_lecture/motor_data.h"
#include "basic_lecture/move_motor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_node");
  
  return 0;
}
