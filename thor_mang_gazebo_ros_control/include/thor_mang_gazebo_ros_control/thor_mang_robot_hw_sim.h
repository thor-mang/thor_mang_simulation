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

#ifndef _GAZEBO_ROS_CONTROL___THOR_MANG_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___THOR_MANG_ROBOT_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/default_robot_hw_sim.h>

#include <hector_gazebo_plugins/sensor_model.h>

// URDF
#include <urdf/model.h>

//FT Sensor Gravity Compensation
#include <vigir_force_torque_compensation_lib/compensation.h>
#include <geometry_msgs/PoseStamped.h>



namespace gazebo_ros_control
{

/**
 * @brief The ThorMangRobotHWSim class provides a ros_control interface
 * for the thor mang robot. It inherits from DefaultHWSim which makes
 * the standard gazebo_ros_control hardware interface available and
 * adds support for the 4 force/torque sensors and IMU used on the robot.
 */
class ThorMangRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:

  enum ftSensorIndex
  {
    R_ARM                         = 0,
    L_ARM                         = 1,
    R_LEG                         = 2,
    L_LEG                         = 3,
    MAXIMUM_NUMBER_OF_FT_SENSORS  = 4
  };

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

protected:

  hardware_interface::ForceTorqueSensorInterface ft_interface_;
  hardware_interface::ImuSensorInterface         imu_interface_;

  // IMU related members
  hardware_interface::ImuSensorHandle::Data imu_data;
  double imu_orientation[4];
  double imu_angular_velocity[3];
  double imu_linear_acceleration[3];
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr imu_link_;

  /// \brief save current body/physics state
  gazebo::math::Quaternion orientation;
  gazebo::math::Vector3 velocity;
  gazebo::math::Vector3 accel;
  gazebo::math::Vector3 rate;
  gazebo::math::Vector3 gravity;

  /// \brief Sensor models
  gazebo::SensorModel3 accelModel;
  gazebo::SensorModel3 rateModel;
  gazebo::SensorModel yawModel;

  /// \brief allow specifying constant xyz and rpy offsets
  gazebo::math::Pose offset_;

  // FT-Sensor related members
  std::string ftSensorUIDs[MAXIMUM_NUMBER_OF_FT_SENSORS];
  std::string ftSensorJoints[MAXIMUM_NUMBER_OF_FT_SENSORS];
  double force_raw[MAXIMUM_NUMBER_OF_FT_SENSORS][3];
  double torque_raw[MAXIMUM_NUMBER_OF_FT_SENSORS][3];
  double force_compensated[MAXIMUM_NUMBER_OF_FT_SENSORS][3];
  double torque_compensated[MAXIMUM_NUMBER_OF_FT_SENSORS][3];
  gazebo::physics::JointPtr ft_joints_[MAXIMUM_NUMBER_OF_FT_SENSORS];
  FTCompensation::Compensation ft_compensation[MAXIMUM_NUMBER_OF_FT_SENSORS];
  ros::Publisher ee_publisher[MAXIMUM_NUMBER_OF_FT_SENSORS];

};

typedef boost::shared_ptr<ThorMangRobotHWSim> ThorMangRobotHWSimPtr;

}

#endif // #ifndef _GAZEBO_ROS_CONTROL___THOR_MANG_ROBOT_HW_SIM_H_
