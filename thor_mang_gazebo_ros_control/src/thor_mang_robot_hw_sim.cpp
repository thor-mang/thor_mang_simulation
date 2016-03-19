//=================================================================================================
// Copyright (c) 2015, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#include <thor_mang_gazebo_ros_control/thor_mang_robot_hw_sim.h>

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

namespace gazebo_ros_control
{

bool ThorMangRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh);

  // Resize vectors to our DOF
  n_dof_ = transmissions.size();
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  //pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  // Initialize values
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if(transmissions[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has no associated joints.");
      continue;
    }
    else if(transmissions[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has more than one joint. Currently the default robot hardware simulation "
        << " interface only supports one.");
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
        "Currently the default robot hardware simulation interface only supports one. Using the first entry!");
      //continue;
    }

    // Add data from transmission
    joint_names_[j] = transmissions[j].joints_[0].name_;
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    const std::string& hardware_interface = joint_interfaces.front();

    // Debug
    ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_names_[j]
      << "' of type '" << hardware_interface << "'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
    if(hardware_interface == "EffortJointInterface")
    {
      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);
      ej_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "PositionJointInterface")
    {
      // Create position joint interface
      joint_control_methods_[j] = POSITION;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "VelocityJointInterface")
    {
      // Create velocity joint interface
      joint_control_methods_[j] = VELOCITY;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_velocity_command_[j]);
      vj_interface_.registerHandle(joint_handle);
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
        << hardware_interface );
      return false;
    }

    // Get the gazebo joint that corresponds to the robot joint.
    //ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    if (!joint)
    {
      ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);

    registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
                        joint_limit_nh, urdf_model,
                        &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                        &joint_effort_limits_[j]);
    //if (joint_control_methods_[j] != EFFORT)
    //{
      // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
      // joint->SetParam("vel") to control the joint.
    //  const ros::NodeHandle nh(model_nh, "/gazebo_ros_control/pid_gains/" +
    //                           joint_names_[j]);
    //  if (pid_controllers_[j].init(nh, true))
    //  {
    //    switch (joint_control_methods_[j])
    //    {
    //      case POSITION:
    //        joint_control_methods_[j] = POSITION_PID;
    //        break;
    //      case VELOCITY:
    //        joint_control_methods_[j] = VELOCITY_PID;
    //        break;
    //    }
    //  }
    //  else
    //  {
        // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
        // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
        // going to be called.
//#if GAZEBO_MAJOR_VERSION > 2
//        joint->SetParam("fmax", 0, joint_effort_limits_[j]);
//#else
//        joint->SetMaxForce(0, joint_effort_limits_[j]);
//#endif
//      }
//    }
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;


  /** register sensors */
  // IMU
  imu_data.name = "pelvis_imu";
  imu_data.frame_id = "pelvis";
  imu_data.orientation = imu_orientation;
  imu_data.angular_velocity = imu_angular_velocity;
  imu_data.linear_acceleration = imu_linear_acceleration;
  hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data);
  imu_interface_.registerHandle(imu_sensor_handle);
  registerInterface(&imu_interface_);

  world_ = parent_model->GetWorld();
  imu_link_ = parent_model->GetLink(imu_data.frame_id);


  ftSensorUIDs[0] = "r_hand";
  ftSensorUIDs[1] = "l_hand";
  ftSensorUIDs[2] = "r_foot";
  ftSensorUIDs[3] = "l_foot";

  ftSensorJoints[0] = "r_arm_wr_p";
  ftSensorJoints[1] = "l_arm_wr_p";
  ftSensorJoints[2] = "r_leg_an_r";
  ftSensorJoints[3] = "l_leg_an_r";

  // FT-Sensors
  for (unsigned int sensorIndex = 0; sensorIndex < MAXIMUM_NUMBER_OF_FT_SENSORS; sensorIndex++)
  {
    hardware_interface::ForceTorqueSensorHandle force_torque_sensor_handle_raw(ftSensorUIDs[sensorIndex] + "_raw", ftSensorUIDs[sensorIndex], force_raw[sensorIndex], torque_raw[sensorIndex]);
    ft_interface_.registerHandle(force_torque_sensor_handle_raw);

    ft_joints_[sensorIndex] = parent_model->GetJoint(ftSensorJoints[sensorIndex]);

    if (!ft_joints_[sensorIndex]){
      ROS_ERROR("Null pointer for joint %s", ftSensorJoints[sensorIndex].c_str());
    }
    else
    {
      ft_joints_[sensorIndex]->SetProvideFeedback(true);

      hardware_interface::ForceTorqueSensorHandle force_torque_sensor_handle_compensated(ftSensorUIDs[sensorIndex], ftSensorUIDs[sensorIndex], force_compensated[sensorIndex], torque_compensated[sensorIndex]);
      ft_interface_.registerHandle(force_torque_sensor_handle_compensated);
    }
  }
  registerInterface(&ft_interface_);

  // load compensation data from parameter server
  for (unsigned int sensorIndex = 0; sensorIndex < MAXIMUM_NUMBER_OF_FT_SENSORS; sensorIndex++)
  {
    ros::NodeHandle nh(ftSensorUIDs[sensorIndex]);
    if (!(ft_compensation[sensorIndex].loadMassComBias(nh) && ft_compensation[sensorIndex].loadHandToSensorOffset(nh, "sensor_offset"))) {
      ROS_WARN_STREAM("Couldn't load complete ft sensor compensation data for " << ftSensorUIDs[sensorIndex] << " in " << nh.getNamespace() << ".");
    }
    ft_compensation[sensorIndex].initGravityPublisher(robot_namespace + "/" + ftSensorUIDs[sensorIndex], ftSensorUIDs[sensorIndex]);
   }

  transforms_.init();

  //DynamixelPro
  dynamixel_pro_controllers_.resize(n_dof_);
  double pos_p, vel_p, vel_i, VelocityLimit, TorqueLimit, positionFactor, velocityFactor, currentFactor;
  for(unsigned int j=0; j < n_dof_; j++)
  {
    if (transmissions[j].actuators_[0].name_ == "H54-200-S500_motor")
    {
      pos_p = 32 * 3.927/10;
      vel_p = 399 * 1.0472/100000 * 200;
      vel_i = 14 * 2.0433/100000000 * 200;
      TorqueLimit = 620.0;
      VelocityLimit = 16600.0;
      positionFactor = 250950.0/M_PI;
      velocityFactor = (502 * 60)/(2*M_PI);
      currentFactor = 33.0/2048;
    }
    else if (transmissions[j].actuators_[0].name_ == "H54-100-S500_motor")
      {
        pos_p = 32 * 3.927/10;
        vel_p = 256 * 1.0472/100000 * 200;
        vel_i = 16 * 2.0433/100000000 * 200;
        TorqueLimit = 310.0;
        VelocityLimit = 17000.0;
        positionFactor = 250950.0/M_PI;
        velocityFactor = (502 * 60)/(2*M_PI);
        currentFactor = 33.0/2048;
      }
      else if (transmissions[j].actuators_[0].name_ == "H42-20-S300_motor")
        {
          pos_p = 32 * 3.927/10;
          vel_p = 440 * 1.0472/100000 * 1;
          vel_i = 40 * 2.0433/100000000 * 1;
          TorqueLimit = 465.0;
          VelocityLimit = 10300.0;
          positionFactor = 151875.0/M_PI;
          velocityFactor = (304 * 60)/(2*M_PI);
          currentFactor = 82.5/2048;
        }
        else
        {
	  ROS_ERROR("ActuatorName %s of Joint-Transmission %s is not known, loading default values.", transmissions[j].actuators_[0].name_.c_str(), transmissions[j].joints_[0].name_.c_str());
          pos_p = 32 * 3.927/10;
          vel_p = 399 * 1.0472/100000 * 200;
          vel_i = 14 * 2.0433/100000000 * 200;
          TorqueLimit = 620.0;
          VelocityLimit = 16600.0;
          positionFactor = 250950.0/M_PI;
          velocityFactor = (502 * 60)/(2*M_PI);
          currentFactor = 33.0/2048;
        }

    control_toolbox::DynamixelPro dynamixel_pro_controller(pos_p,vel_p,vel_i,VelocityLimit,TorqueLimit,positionFactor,velocityFactor,currentFactor);

    dynamixel_pro_controllers_[j] = dynamixel_pro_controller;
    
    const ros::NodeHandle nh(model_nh, joint_names_[j] + "/dynamixel_pro_controller_gains/");
    dynamixel_pro_controllers_[j].init(nh, true);
  }

  return true;
}

void ThorMangRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  // Call parent read (dealing with joint interfaces)
  DefaultRobotHWSim::readSim(time, period);

/*   // Transforms
  for (unsigned int i = 0; i < sim_joints_.size(); i++) {
    // Gazebo has an interesting API...
    // Only works for revolute joints!
    joint_position_[i] += angles::shortest_angular_distance(joint_position_[i],
                            sim_joints_[i]->GetAngle(0).Radian());
    transforms_.updateState(sim_joints_[i]->GetName(), joint_position_[i]);
  }



  // IMU (largely copied from hector_gazebo_ros_imu)
  double dt = period.toSec();

  // Get Pose/Orientation
  gazebo::math::Pose pose = imu_link_->GetWorldPose();
  gazebo::math::Quaternion rot = this->offset_.rot * pose.rot;
  rot.Normalize();

  // get Gravity
  gravity = world_->GetPhysicsEngine()->GetGravity();
  double gravity_length = gravity.GetLength();
  ROS_DEBUG_NAMED("gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.x, gravity.y, gravity.z);

  // get Acceleration and Angular Rates
  // the result of GetRelativeLinearAccel() seems to be unreliable (sum of forces added during the current simulation step)?
  //accel = myBody->GetRelativeLinearAccel(); // get acceleration in body frame
  gazebo::math::Vector3 temp = imu_link_->GetWorldLinearVel(); // get velocity in world frame
  if (dt > 0.0) accel = rot.RotateVectorReverse((temp - velocity) / dt - gravity);
  velocity = temp;

  // calculate angular velocity from delta quaternion
  // note: link->GetRelativeAngularVel() sometimes return nan?
  // rate  = link->GetRelativeAngularVel(); // get angular rate in body frame
  gazebo::math::Quaternion delta = this->orientation.GetInverse() * rot;
  this->orientation = rot;
  if (dt > 0.0) {
    rate = 2.0 * acos(std::max(std::min(delta.w, 1.0), -1.0)) * gazebo::math::Vector3(delta.x, delta.y, delta.z).Normalize() / dt;
  }

  // update sensor models
  accel = accelModel(accel, dt);
  rate  = rateModel(rate, dt);
  yawModel.update(dt);
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Current bias errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
                  accelModel.getCurrentBias().x, accelModel.getCurrentBias().y, accelModel.getCurrentBias().z,
                  rateModel.getCurrentBias().x, rateModel.getCurrentBias().y, rateModel.getCurrentBias().z,
                  yawModel.getCurrentBias());
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Scale errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
                  accelModel.getScaleError().x, accelModel.getScaleError().y, accelModel.getScaleError().z,
                  rateModel.getScaleError().x, rateModel.getScaleError().y, rateModel.getScaleError().z,
                  yawModel.getScaleError());


  // apply accelerometer and yaw drift error to orientation (pseudo AHRS)
  gazebo::math::Vector3 accelDrift = pose.rot.RotateVector(accelModel.getCurrentBias());
  double yawError = yawModel.getCurrentBias();
  gazebo::math::Quaternion orientationError(
        gazebo::math::Quaternion(cos(yawError/2), 0.0, 0.0, sin(yawError/2)) *                                         // yaw error
        gazebo::math::Quaternion(1.0, 0.5 * accelDrift.y / gravity_length, 0.5 * -accelDrift.x / gravity_length, 0.0)  // roll and pitch error
        );

  orientationError.Normalize();
  rot = orientationError * rot;

  imu_orientation[0] = rot.x;
  imu_orientation[1] = rot.y;
  imu_orientation[2] = rot.z;
  imu_orientation[3] = rot.w;

  imu_angular_velocity[0] = rate.x;
  imu_angular_velocity[1] = rate.y;
  imu_angular_velocity[2] = rate.z;

  imu_linear_acceleration[0] = accel.x;
  imu_linear_acceleration[1] = accel.y;
  imu_linear_acceleration[2] = accel.z;

  Eigen::Affine3d imu_orient(Eigen::Quaternion<double>(imu_orientation[3], imu_orientation[0], imu_orientation[1], imu_orientation[2]));
  imu_orient.translation()  = Eigen::Vector3d::Zero();
  transforms_.updateRootTransform(imu_orient); */

  /*
   // fill in covariance matrix
   imuMsg.orientation_covariance[8] = yawModel.gaussian_noise*yawModel.gaussian_noise;
   if (gravity_length > 0.0) {
     imuMsg.orientation_covariance[0] = accelModel.gaussian_noise.x*accelModel.gaussian_noise.x/(gravity_length*gravity_length);
     imuMsg.orientation_covariance[4] = accelModel.gaussian_noise.y*accelModel.gaussian_noise.y/(gravity_length*gravity_length);
   } else {
     imuMsg.orientation_covariance[0] = -1;
     imuMsg.orientation_covariance[4] = -1;
   }
  */

/*   // FT-Sensors
  for (unsigned int sensorIndex = 0; sensorIndex < MAXIMUM_NUMBER_OF_FT_SENSORS; sensorIndex++)
  {
    gazebo::physics::JointWrench wrench;
    gazebo::math::Vector3 torque;
    gazebo::math::Vector3 force;

    // FIXME: Should include options for different frames and measure directions
    // E.g: https://bitbucket.org/osrf/gazebo/raw/default/gazebo/sensors/ForceTorqueSensor.hh
    // Get force torque at the joint
    // The wrench is reported in the CHILD <frame>
    // The <measure_direction> is child_to_parent
    wrench = ft_joints_[sensorIndex]->GetForceTorque(0);
    force = wrench.body2Force;
    torque = wrench.body2Torque;

    force_raw[sensorIndex][0] = force.x; //+ this->GaussianKernel(0, this->gaussian_noise_);
    force_raw[sensorIndex][1] = force.y; //+ this->GaussianKernel(0, this->gaussian_noise_);
    force_raw[sensorIndex][2] = force.z; //+ this->GaussianKernel(0, this->gaussian_noise_);
    torque_raw[sensorIndex][0] = torque.x; //+ this->GaussianKernel(0, this->gaussian_noise_);
    torque_raw[sensorIndex][1] = torque.y; //+ this->GaussianKernel(0, this->gaussian_noise_);
    torque_raw[sensorIndex][2] = torque.z; //+ this->GaussianKernel(0, this->gaussian_noise_);

    // FT Compensation
    Eigen::Matrix3d world_gripper_rot = (transforms_.getRootTransform().rotation() * transforms_.getTransform(ftSensorUIDs[sensorIndex]).rotation()).inverse();
    ft_compensation[sensorIndex].setWorldGripperRotation(world_gripper_rot);
    FTCompensation::Vector6d ft_raw;
    FTCompensation::Vector6d ft_compensated;

    ft_raw[0] = force_raw[sensorIndex][0];
    ft_raw[1] = force_raw[sensorIndex][1];
    ft_raw[2] = force_raw[sensorIndex][2];
    ft_raw[3] = torque_raw[sensorIndex][0];
    ft_raw[4] = torque_raw[sensorIndex][1];
    ft_raw[5] = torque_raw[sensorIndex][2];
    ft_compensation[sensorIndex].zeroAndCompensate(ft_raw, ft_compensated);
    force_compensated[sensorIndex][0] = ft_compensated[0];
    force_compensated[sensorIndex][1] = ft_compensated[1];
    force_compensated[sensorIndex][2] = ft_compensated[2];
    torque_compensated[sensorIndex][0] = ft_compensated[3];
    torque_compensated[sensorIndex][1] = ft_compensated[4];
    torque_compensated[sensorIndex][2] = ft_compensated[5];
  } */

  //DynamixelPro
  for(unsigned int j=0; j < n_dof_; j++)
  {
    dynamixel_pro_controllers_[j].updateStates(joint_position_[j], joint_velocity_[j], joint_effort_[j]);
  }
}

void ThorMangRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  if (e_stop_active_)
  {
    if (!last_e_stop_active_)
    {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  }
  else
  {
    last_e_stop_active_ = false;
  }

  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);

  for(unsigned int j=0; j < n_dof_; j++)
  {

if (joint_names_[j] == "dynamixel_joint")
{
//dynamixel_pro_controllers_[j].printValues();
ROS_INFO_STREAM_NAMED("DynamixelPro","Values of DynamixelPro Class BEFORE:\n"
    << "  Joint-Name: " << joint_names_[j] << "\n"
    << "  Velocity: " << joint_velocity_[j] << "\n"
    << "  Force: " << sim_joints_[j]->GetForce(0u) << "\n"
  );
}

    switch (joint_control_methods_[j])
    {
      case EFFORT:
        {
                if ((joint_names_[j] == "r_f0_j0") || (joint_names_[j] == "r_f1_j0") || (joint_names_[j] == "l_f0_j0") || (joint_names_[j] == "l_f1_j0"))
                {
                        //Simulation is done by VT_Hand Plugin
                }
                else 
                {
                        const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
                        sim_joints_[j]->SetForce(0, effort);
                }

        }
        break;

      case POSITION:
        {
                if ((joint_names_[j] == "r_f0_j0") || (joint_names_[j] == "r_f1_j0") || (joint_names_[j] == "l_f0_j0") || (joint_names_[j] == "l_f1_j0"))
                {
                        //Simulation is done by VT_Hand Plugin
                }
                else 
                {
			double error;
         		switch (joint_types_[j])
          		{
            		case urdf::Joint::REVOLUTE:
              		angles::shortest_angular_distance_with_limits(joint_position_[j],
                                                            joint_position_command_[j],
                                                            joint_lower_limits_[j],
                                                            joint_upper_limits_[j],
                                                            error);
              		break;
            		case urdf::Joint::CONTINUOUS:
              		error = angles::shortest_angular_distance(joint_position_[j],
                                                        joint_position_command_[j]);
              		break;
            		default:
              		error = joint_position_command_[j] - joint_position_[j];
          		}

          		const double effort_limit = joint_effort_limits_[j];
          		const double effort = clamp(dynamixel_pro_controllers_[j].computePositionCommand(joint_position_command_[j], period),
                                      -effort_limit, effort_limit);

          		sim_joints_[j]->SetForce(0, effort);
                }
        }
        break;

      case VELOCITY:
//#if GAZEBO_MAJOR_VERSION > 2
//        sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
//#else
//        sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
//#endif
//        break;

      case VELOCITY_PID:
        double error;
        if (e_stop_active_)
          error = -joint_velocity_[j];
        else
          error = joint_velocity_command_[j] - joint_velocity_[j];
        const double effort_limit = joint_effort_limits_[j];
        const double effort = clamp(dynamixel_pro_controllers_[j].computeVelocityCommand(2, period),
                                    -effort_limit, effort_limit);
        sim_joints_[j]->SetForce(0, effort);
        break;
    }

if (joint_names_[j] == "dynamixel_joint")
{
dynamixel_pro_controllers_[j].printValues();
ROS_INFO_STREAM_NAMED("DynamixelPro","Values of DynamixelPro Class AFTER:\n"
    << "  Joint-Name: " << joint_names_[j] << "\n"
    << "  Velocity: " << joint_velocity_[j] << "\n"
    << "  Force: " << sim_joints_[j]->GetForce(0u) << "\n"
  );
}	

  }
}

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::ThorMangRobotHWSim, gazebo_ros_control::RobotHWSim)
