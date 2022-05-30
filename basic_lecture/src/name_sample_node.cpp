/**
 * @file name_sample_node.cpp
 */

#include "ros/this_node.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

using namespace std;

void usage()
{
  cerr << "To set namespace to ROS node by using rosrun, specify __ns=<namespace> " << endl;
  cerr << "To set a name to ROS node by using rosrun, specify __name=<name> " << endl;
}

void callback(const std_msgs::String::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
  usage();

  ros::init(argc, argv, "name_sample_node");

  // NodeHandle is generated with each namespaces
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::NodeHandle sample_nh("sample");

  // Subscribers are set by using NodeHandle
  ros::Subscriber base_sub = nh.subscribe("base_name_topic", 1, callback);
  ros::Subscriber private_sub = private_nh.subscribe("private_name_topic", 1, callback);
  ros::Subscriber sample_sub = sample_nh.subscribe("sample_name_topic", 1, callback);

  // Tilde means private namespace, but it cannot be used with NodeHandle
  //ros::Subscriber private_sub = nh.subscribe("~private_name_topic", 1, callback); // Error!

  // Set parameters by using NodeHandle
  nh.setParam("base_name_param", "base_name");
  private_nh.setParam("private_name_param", "private_name");
  sample_nh.setParam("sample_name_param", "sample_name");

  // Set parameters without NodeHandle
  ros::param::set("default_namespace_param", "default_name");
  ros::param::set("~tilde_namespace_param", "tilde_name");

  // Get parameters by using NodeHandle
  string base_launch_str_param, private_launch_str_param;
  string base_param_info, private_param_info;
  if (nh.getParam("launch_str_param", base_launch_str_param))
  {
    stringstream ss;
    ss << nh.getNamespace() << "/launch_str_param <base parameter> should be loaded: ";
    ss << "[ " << base_launch_str_param << " ]";
    base_param_info = ss.str();
  }
  if (private_nh.getParam("launch_str_param", private_launch_str_param))
  {
    stringstream ss;
    ss << private_nh.getNamespace() << "/launch_str_param <private parameter> should be loaded: ";
    ss << "[ " << private_launch_str_param << " ]";
    private_param_info = ss.str();
  }

  // Output information at once
  cout << "--------------------------" << endl
       << "Node Name: " << ros::this_node::getName() << endl
       << "default node's namespace is [ " << nh.getNamespace() << " ]" << endl
       << "private node's namespace is [ " << private_nh.getNamespace() << " ]" << endl
       << "sample node's namespace is [ " << sample_nh.getNamespace() << " ]" << endl
       << base_param_info << endl
       << private_param_info << endl
       << "$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl << endl;

  ros::spin();

  return 0;
}
