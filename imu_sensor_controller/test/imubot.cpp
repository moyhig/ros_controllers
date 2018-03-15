#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "imubot.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imubot");
  ros::NodeHandle nh;

  Imubot<> robot;
  ROS_WARN_STREAM("period: " <<robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();
  
  return 0;
}
