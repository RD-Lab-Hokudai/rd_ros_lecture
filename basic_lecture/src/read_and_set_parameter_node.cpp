/**
 * @file read_and_set_parameter_node.cpp
 * @brief Read parameter; str_parameter and num_parameter, and set other parameters
 */

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_and_set_parameter_node");
  ros::NodeHandle n;

  // Parameter serverからパラメータを読み込む
  std::string param1_value;
  if (n.getParam("str_parameter", param1_value))
  {
    std::string message ="parameter:str_parameter is gotten, %s";
    ROS_INFO(message.c_str(), param1_value.c_str());
  }
  else
  {
    ROS_ERROR("parameter:str_parameter couldn't be gotten");
  }

  double param2_value;
  if (n.getParam("num_parameter", param2_value))
  {
    std::string message ="parameter:num_parameter is gotten, %f";
    ROS_INFO(message.c_str(), param2_value);
  }
  else
  {
    ROS_ERROR("parameter:num_parameter couldn't be gotten");
  }

  // parameter をパラメータサーバに設定する
  n.setParam("int_set_param", 10);
  n.setParam("string_set_param", "string_data");
  n.setParam("bool_set_param", false);
  ROS_INFO("Set parameters to parameter server");
  return 0;
}
