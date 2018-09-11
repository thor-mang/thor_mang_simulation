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
  if(!kdl_parser::treeFromParam(robotDescription, robot_))
  {
    ROS_ERROR("[THOR::FakeFTSensors]: Failed to construct KDL tree of robot description");
    return false;
  }

  return true;
}

void FakeFTSensor::process()
{
  calcRobotDynamics();
}

void FakeFTSensor::reset()
{

}

void FakeFTSensor::calcRobotDynamics()
{
  KDL::Vector grav(0, 0, -9.81);
  KDL::Chain leftLegChain;
  robot_.getChain("pelvis", "leftFT", leftLegChain);
  KDL::Chain rightLegChain;
  robot_.getChain("pelvis", "rightFT", rightLegChain);

  KDL::ChainIdSolver_RNE leftDynamics(leftLegChain, grav);
  KDL::ChainIdSolver_RNE rightDynamics(rightLegChain, grav);

  KDL::JntArray q, q_dot, q_dotdot;
  KDL::JntArray torques;
  KDL::Wrenches f_ext;

  leftDynamics.CartToJnt(q, q_dot, q_dotdot, f_ext, torques);

  for(int i = 0; i < q_dot.rows(); i++)
  {
    ROS_ERROR("Geschwindigkeit: %f", q_dot(1));
  }

}

} // namespace

