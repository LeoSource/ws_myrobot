#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mqtt_client/PublishData.h"
#include "MqttTransition.h"


void MqttPublishCallback(const mqtt_client::PublishData::ConstPtr& msg)
{
  ROS_INFO("%s",msg->task.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqtt_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("keyboard_control",1000,MqttPublishCallback);

  mqtt::async_client cli(SERVER_ADDRESS,CLIENT_ID);
  mqtt::connect_options connOpts;
  connOpts.set_clean_session(false);
  callback cb(cli,connOpts);
  cli.set_callback(cb);
  MQTTTransition mqtt_trans(cli,connOpts,cb);
  mqtt_trans.Connect();

  ros::spin();


  return 0;
}