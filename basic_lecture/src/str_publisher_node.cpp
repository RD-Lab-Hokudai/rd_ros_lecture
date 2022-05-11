/**
 * @file str_publisher_node.cpp
 * @brief /str_data をpublish
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  // 引数argc, argvはROSのtopic名などをrenameするために使われる．
  // ROSの機能を実行する前に最初に呼んでおく必要がある．
  ros::init(argc, argv, "str_publisher_node");

  // Publisherの登録など，ROSのシステムに対する起点となる．
  ros::NodeHandle n;

  // ros::Publisherはtopicをpublishするときに使うクラス
  // n.advertise()で，s_data_pubは
  // std_msgs/Stringというメッセージ(型)で"str_data"という名前のtopicとして
  // データをpublishするものと設定している．
  // 引数にトピック名とデータキューの数を渡す．
  ros::Publisher s_data_pub = n.advertise<std_msgs::String>("str_data", 1);

  // ros::Rateは以下のようにwhileで周期毎に動作させたいコードに用いる
  // 引数はHz，ros::Rate::sleep()で目標の周波数になるように調整される．
  // ただしROSはリアルタイムシステムではないので，時間のかかる実行内容に
  // よって目標の周波数にならないこともある．
  ros::Rate loop_rate(10);

  // ros::Rateによって周期的にデータを送る．
  // 一般的にロボットの動作制御は10ms-100msで周期的に行われる
  while (ros::ok())
  {
    // ROSのメッセージ，C++だとクラスとなる．
    std_msgs::String msg; // 文字列を格納できるの変数
    std::string s_data = "string data, Hello World!";
    // 端末で $ rosmsg show std_msgs/Stringを実行してみよう．
    // string data と出てくるはずだ．
    // これはプリミティブな型stringのdataという名前の変数(メンバ)である．
    msg.data = s_data;

    // ros::Publisher::publish()で実際にデータを指定のトピックにpublishする
    s_data_pub.publish(msg); //予め指定したstd_msgs::String型のみ

    //ros::spinOnce(); //もし他にsubscriberを設定している場合は必要．

    loop_rate.sleep();
  }
  return 0;
}
