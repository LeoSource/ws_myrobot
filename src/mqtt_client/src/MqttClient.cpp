#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MqttTransition.h"

// void pubcallback(const mqtt_client::PublishData::ConstPtr& msg)
// {
//   ROS_INFO("%s",msg->task.c_str());
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher armdata_pub = n.advertise<mqtt_client::SubscribeData>("xarm_data", 1000);
  int freq_mqtt;
  n.getParam("freq_mqtt_receive",freq_mqtt);
  ros::Rate loop_rate(freq_mqtt);

  mqtt::async_client cli(SERVER_ADDRESS,CLIENT_ID);
  mqtt::connect_options connOpts;
  connOpts.set_clean_session(false);
  callback cb(cli,connOpts);
  cli.set_callback(cb);
  MqttTransition mqtt_trans(cli,connOpts,cb);
  mqtt_trans.Connect();

  ros::Subscriber sub = n.subscribe("keyboard_control",1000,MqttTransition::PublishCallback);

  mqtt_client::SubscribeData arm_data;
  while (ros::ok())
  {
    arm_data.header.stamp = ros::Time::now();
    mqtt_trans.GetJointPos(arm_data.joint_position);
    armdata_pub.publish(arm_data);

    ros::spinOnce();
    loop_rate.sleep();

    // if(!mqtt_trans.IsConnected())
    //   ROS_INFO("ERROR: Unable to connect to MQTT server");
  }

  mqtt_trans.DisConnect();

  return 0;
}