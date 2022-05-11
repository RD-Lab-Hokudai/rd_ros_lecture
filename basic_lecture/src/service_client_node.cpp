/**
 * @file service_client_node.cpp
 * @brief
 */


#include <ros/ros.h>
#include <std_srvs/SetBool.h> // Serviceの型のインクルード

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client_node");
  ros::NodeHandle n;

  // Serviceを実行してもらい，その結果を受け取るためのclientクラス
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("service_SetBool_example");

  std_srvs::SetBool srv;
  srv.request.data = true;
  if (client.call(srv))
  {
    ROS_INFO_STREAM("Response of service: " << srv.response);
  }
  else
  {
    ROS_ERROR("Failed to call service service_SetBool_example");
    return 1;
  }

  return 0;
}

