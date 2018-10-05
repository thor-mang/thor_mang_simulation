#include <thor_mang_fake_ft_sensor/thor_mang_fake_ft_sensor.h>

namespace thormang3
{
FakeFTSensor::FakeFTSensor(ros::NodeHandle& nh)
  : nh_(nh)
{

}

FakeFTSensor::FakeFTSensor()
{

}

FakeFTSensor::~FakeFTSensor()
{

}

bool FakeFTSensor::initialize(std::string robotParamName)
{
  std::string robotDescription;
  if(!nh_.getParam(robotParamName, robotDescription))
  {
    ROS_ERROR("[THOR::FakeFTSensors]: No Param named %s was found", robotParamName.c_str());
    return false;
  }
  if(!kdl_parser::treeFromString(robotDescription, robot_))
  {
    ROS_ERROR("[THOR::FakeFTSensors]: Failed to construct KDL tree of robot description");
    return false;
  }

  jointStateSub_ = nh_.subscribe<sensor_msgs::JointState>("/johnny5/joints/joint_states", 1000, &FakeFTSensor::jointStateCallback, this);

  leftFTSensorPub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/johnny5/sensor/ft/left_foot/faked", 1000, this);
  rightFTSensorPub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/johnny5/sensor/ft/right_foot/faked", 1000, this);

  return true;
}

void FakeFTSensor::process()
{

}

void FakeFTSensor::reset()
{

}

void FakeFTSensor::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{

}

} // namespace

