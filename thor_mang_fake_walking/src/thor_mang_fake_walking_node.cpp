#include <ros/ros.h>
#include <thor_mang_fake_walking/thor_mang_fake_walking.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_walking_node");
  ROS_DEBUG("Starting Fake Walking Node");
  ros::NodeHandle nh("");
  fake_walking::FakeWalking fake_walking(nh);
  ros::spin();
  exit(0);

}


