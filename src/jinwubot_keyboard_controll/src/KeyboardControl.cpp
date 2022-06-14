#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include "mqtt_client/PublishData.h"

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

class KeyboardReader
{
public:
  KeyboardReader(): kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char * c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }
private:
  int kfd;
  struct termios cooked;
};

KeyboardReader input;

class KeyboardController
{
public:
  KeyboardController();
  void keyLoop();

private:
  ros::NodeHandle _nh;
  std::string _task_name;
  ros::Publisher _keyboard_pub;
  
};

KeyboardController::KeyboardController()
{
  _keyboard_pub = _nh.advertise<mqtt_client::PublishData>("keyboard_control", 100);
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  KeyboardController teleop_xarm;

  signal(SIGINT,quit);

  teleop_xarm.keyLoop();
  quit(0);
  
  return(0);
}


void KeyboardController::keyLoop()
{
  char c;
  bool dirty=false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle. 'q' to quit.");


  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_DEBUG("LEFT");
        _task_name = "left";
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_DEBUG("RIGHT");
        _task_name = "right";
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_DEBUG("UP");
        _task_name = "up";
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("DOWN");
        _task_name = "down";
        dirty = true;
        break;
      case KEYCODE_B:
        ROS_DEBUG("B");
        _task_name = "home";
        dirty = true;
        break;
      case KEYCODE_C:
        ROS_DEBUG("C");
        _task_name = "get_to_standby";
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("D");
        _task_name = "initial";
        dirty = true;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("quit");
        return;
    }
   
    mqtt_client::PublishData pub_data;
    pub_data.header.stamp = ros::Time::now();
    pub_data.task = _task_name;
    if(dirty ==true)
    {
      _keyboard_pub.publish(pub_data);
      ROS_INFO("%s",pub_data.task.c_str());
      dirty=false;
    }
  }


  return;
}


