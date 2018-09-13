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
  return true;
}

void FakeFTSensor::process()
{
  calcRobotLegDynamics();
}

void FakeFTSensor::reset()
{

}

void FakeFTSensor::calcRobotLegDynamics()
{
  KDL::Vector grav(0, 0, -9.81);
  KDL::Chain leftLegChain;
  robot_.getChain("pelvis_link", "l_leg_an_r_link", leftLegChain);
  KDL::Chain rightLegChain;
  robot_.getChain("pelvis_link", "r_leg_an_r_link", rightLegChain);

  KDL::ChainIdSolver_RNE leftDynamics(leftLegChain, grav);
  KDL::ChainIdSolver_RNE rightDynamics(rightLegChain, grav);

  //leftDynamics.CartToJnt(leftLeg_.pos, leftLeg_.vel, leftLeg_.acc, leftLeg_.force, leftLeg_.torque);
  //rightDynamics.CartToJnt(rightLeg_.pos, rightLeg_.vel, rightLeg_.acc, rightLeg_.force, rightLeg_.torque);

}

void FakeFTSensor::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{

}

} // namespace

