#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"view");

  ros::NodeHandle n;

  sensor_msgs::JointState joint_state;
  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    double jpos1;
    if(count>100)
      jpos1 = M_PI/2;
    else
      jpos1 = 0;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"joint0","joint1","joint2","joint3","joint4","joint5"};
    joint_state.velocity = {0,0,0,0,0,0};
    joint_state.effort = {0,0,0,0,0,0};
    joint_state.position = {jpos1,0,0,0,0,0};

    ROS_INFO("%s", joint_state.name[3].c_str());

    joint_state_pub.publish(joint_state);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}