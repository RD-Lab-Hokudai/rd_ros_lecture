/**
 * @file name_sample_node.cpp
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

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
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::NodeHandle sample_nh("sample");
  cout << "default node's namespace is [ " << nh.getNamespace() << " ]" << endl;
  cout << "private node's namespace is [ " << private_nh.getNamespace() << " ]" << endl;
  cout << "sample node's namespace is [ " << sample_nh.getNamespace() << " ]" << endl;

  ros::Subscriber base_sub = nh.subscribe("base_name_topic", 1, callback);
  ros::Subscriber private_sub = private_nh.subscribe("private_name_topic", 1, callback);
  ros::Subscriber sample_sub = sample_nh.subscribe("sample_name_topic", 1, callback);

  //ros::Subscriber private_sub = nh.subscribe("~private_name_topic", 1, callback); // Error!

  nh.setParam("base_name_param", "base_name");
  private_nh.setParam("private_name_param", "private_name");
  sample_nh.setParam("sample_name_param", "sample_name");

  ros::param::set("default_namespace_param", "default_name");
  ros::param::set("~tilde_namespace_param", "tilde_name");

  ros::spin();

  return 0;
}
