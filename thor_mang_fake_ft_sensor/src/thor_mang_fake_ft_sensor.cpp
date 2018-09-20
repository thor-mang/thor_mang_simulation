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

  joint_names_.push_back("r_leg_hip_y");
  joint_names_.push_back("r_leg_hip_r");
  joint_names_.push_back("r_leg_hip_p");
  joint_names_.push_back("r_leg_kn_p");
  joint_names_.push_back("r_leg_an_p");
  joint_names_.push_back("r_leg_an_r");

  joint_names_.push_back("l_leg_hip_y");
  joint_names_.push_back("l_leg_hip_r");
  joint_names_.push_back("l_leg_hip_p");
  joint_names_.push_back("l_leg_kn_p");
  joint_names_.push_back("l_leg_an_p");
  joint_names_.push_back("l_leg_an_r");

  position_ = KDL::JntArray(joint_names_.size());
  velocity_ = KDL::JntArray(joint_names_.size());
  acceleration_ = KDL::JntArray(joint_names_.size());
  torques_left_ = KDL::JntArray(joint_names_.size() / 2);
  torques_right_ = KDL::JntArray(joint_names_.size() / 2);

  for(int i = 0; i < joint_names_.size() / 2; i++) force_.push_back(KDL::Wrench());

  jointStateSub_ = nh_.subscribe<sensor_msgs::JointState>("/johnny5/joints/joint_states", 1000, &FakeFTSensor::jointStateCallback, this);
  leftFTSensorPub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/johnny5/sensor/ft/left_foot/faked", 1000, this);
  rightFTSensorPub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/johnny5/sensor/ft/right_foot/faked", 1000, this);
  return true;
}

void FakeFTSensor::process()
{
  KDL::Vector grav(0, 0, -9.81);
  KDL::Chain leftLegChain;
  robot_.getChain("pelvis_link", "l_leg_an_r_link", leftLegChain);
  KDL::Chain rightLegChain;
  robot_.getChain("pelvis_link", "r_leg_an_r_link", rightLegChain);

  KDL::ChainIdSolver_RNE leftDynamics(leftLegChain, grav);
  KDL::ChainIdSolver_RNE rightDynamics(rightLegChain, grav);

  int right_error = rightDynamics.CartToJnt(trunk(position_, 0, 6), trunk(velocity_, 0, 6), trunk(acceleration_, 0, 6), force_, torques_right_);
  int left_error = leftDynamics.CartToJnt(trunk(position_, 6, 12), trunk(velocity_, 6, 12), trunk(acceleration_, 6, 12), force_, torques_left_);

  if(right_error != 0)
    ROS_ERROR_THROTTLE(1.0, "Dynamics calculation in the right leg failed with error code %d", right_error);
  if(left_error != 0)
    ROS_ERROR_THROTTLE(1.0, "Dynamics calculation in the left leg failed with error code %d", left_error);

  if(right_error == 0)
  {
    geometry_msgs::WrenchStamped rightFTValue;
    rightFTValue.header.stamp = ros::Time::now();
    rightFTValue.header.frame_id = "pelvis_link";
    rightFTValue.wrench.torque.x = torques_right_(5);
    rightFTValue.wrench.torque.y = 0;
    rightFTValue.wrench.torque.z = 0;
    rightFTSensorPub_.publish(rightFTValue);
  }
  if(left_error == 0)
  {
    geometry_msgs::WrenchStamped leftFTValue;
    leftFTValue.header.stamp = ros::Time::now();
    leftFTValue.header.frame_id = "pelvis_link";
    leftFTValue.wrench.torque.x = torques_left_(5);
    leftFTValue.wrench.torque.y = 0;
    leftFTValue.wrench.torque.z = 0;
    leftFTSensorPub_.publish(leftFTValue);
  }

}

void FakeFTSensor::reset()
{
  position_ = KDL::JntArray(joint_names_.size());
  velocity_ = KDL::JntArray(joint_names_.size());
  acceleration_ = KDL::JntArray(joint_names_.size());
  torques_left_ = KDL::JntArray(joint_names_.size() / 2);
  torques_right_ = KDL::JntArray(joint_names_.size() / 2);

  for(int i = 0; i < joint_names_.size() / 2; i++) force_.push_back(KDL::Wrench());
}

void FakeFTSensor::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  for(int i = 0; i < joint_names_.size(); i++)
  {
    auto it = std::find(joint_states->name.begin(), joint_states->name.end(), joint_names_[i]);
    if(it == joint_states->name.end())
    {
      ROS_ERROR("Joint name %s was not found in the actual robot tree", joint_names_[i].c_str());
      break;
    } else {
      position_(i) = joint_states->position[std::distance(joint_states->name.begin(), it)];
      velocity_(i) = joint_states->velocity[std::distance(joint_states->name.begin(), it)];
      acceleration_(i) = joint_states->effort[std::distance(joint_states->name.begin(), it)];
    }
  }
}

KDL::JntArray FakeFTSensor::trunk(KDL::JntArray generalJoints, int start, int end)
{
  KDL::JntArray ret(end - start);
  for(int i = start; i < end; i++) ret(i-start) = generalJoints(i);

  return ret;
}

} // namespace

