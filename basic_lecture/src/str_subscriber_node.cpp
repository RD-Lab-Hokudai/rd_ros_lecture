/**
 * @file str_subscriber_node.cpp
 * @brief /str_data をsubscribe
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

// 指定のtopicが得られた時呼び出されるコールバック関数
// topicを通して他のnodeから得られたデータを処理する．
// 引数のmsgに購読されたデータが入っている
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // ROS_INFOはlog出力用のマクロ．端末画面もしくはROSのlogファイルに出力
  ROS_INFO("Got next message --> %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "str_subscriber_node");
  ros::NodeHandle n;

  // ros::Subscriberはtopicのsubscribeとそのときに発生させるコールバック関数を
  // 設定するために必要なクラス．
  // 引数に，topic名とデータキュー，その時の関数ポインタを渡す．
  ros::Subscriber s_data_sub = n.subscribe("str_data", 1, chatterCallback);

  // コールバックの待受状態になる．
  // どこかのnodeから購読対象になっているtopic がpublishされると，
  // ros::Subscriberに設定したコールバック関数が呼ばれる．
  // nodeが止められるまで，待ち受け状態は続く
  ros::spin();

  return 0;
}
