#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "mqtt_client/SubscribeData.h"


std::vector<double> jpos,pre_jpos;

void JointSateCallback(const mqtt_client::SubscribeData::ConstPtr& msg)
{
  jpos = msg->joint_position;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"view");
  ros::NodeHandle n;
  sensor_msgs::JointState joint_state;
  ros::Publisher joint_state_pub = 
                    n.advertise<sensor_msgs::JointState>("joint_states", 1000);
  int display_freq,mqtt_freq;
  n.getParam("freq_mqtt_receive",mqtt_freq);
  n.getParam("freq_display",display_freq);
  ros::Rate loop_rate(display_freq);
  ros::Subscriber sub = n.subscribe("xarm_data",1000,JointSateCallback);

  int freq_ratio = display_freq/mqtt_freq;
  int count = 0;
  while (ros::ok())
  {
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"joint0","joint1","joint2","joint3","joint4","joint5"};
    joint_state.velocity = {0,0,0,0,0,0};
    joint_state.effort = {0,0,0,0,0,0};
      
    if(pre_jpos.empty())
      joint_state.position = jpos;
    else
    {
      for(int idx=0;idx<6;idx++)
      {
        double dir_len = (jpos[idx]-pre_jpos[idx])/freq_ratio;
        joint_state.position.at(idx) = pre_jpos[idx]+count*dir_len;
      }
    }

    joint_state_pub.publish(joint_state);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if(count==freq_ratio)
    {
      count=0;
      pre_jpos = jpos;
    }

  }

  return 0;
}