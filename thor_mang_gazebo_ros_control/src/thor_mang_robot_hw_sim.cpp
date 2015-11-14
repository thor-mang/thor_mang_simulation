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

namespace gazebo_ros_control
{

bool ThorMangRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // Call parent init (dealing with joint interfaces)
  if (!DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions)){
    return false;
  }

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

  ftSensorJoints[0] = "r_wrist_yaw2";
  ftSensorJoints[1] = "l_wrist_yaw2";
  ftSensorJoints[2] = "r_ankle_roll";
  ftSensorJoints[3] = "l_ankle_roll";

  // FT-Sensors
  for (unsigned int sensorIndex = 0; sensorIndex < MAXIMUM_NUMBER_OF_FT_SENSORS; sensorIndex++)
  {
    hardware_interface::ForceTorqueSensorHandle force_torque_sensor_handle_raw(ftSensorUIDs[sensorIndex] + "_raw", ftSensorUIDs[sensorIndex], force_raw[sensorIndex], torque_raw[sensorIndex]);
    ft_interface_.registerHandle(force_torque_sensor_handle_raw);

    ft_joints_[sensorIndex] = parent_model->GetJoint(ftSensorJoints[sensorIndex]);

    if (!ft_joints_[sensorIndex]){
      ROS_ERROR("Null pointer for joint %s", ftSensorJoints[sensorIndex].c_str());
    }

    ft_joints_[sensorIndex]->SetProvideFeedback(true);

    hardware_interface::ForceTorqueSensorHandle force_torque_sensor_handle_compensated(ftSensorUIDs[sensorIndex], ftSensorUIDs[sensorIndex], force_compensated[sensorIndex], torque_compensated[sensorIndex]);
    ft_interface_.registerHandle(force_torque_sensor_handle_compensated);
  }
  registerInterface(&ft_interface_);

  // load compensation data from parameter server
  for (unsigned int sensorIndex = 0; sensorIndex < MAXIMUM_NUMBER_OF_FT_SENSORS; sensorIndex++)
  {
    ros::NodeHandle nh(ftSensorUIDs[sensorIndex]);
    if (!(ft_compensation[sensorIndex].loadMassComBias(nh) && ft_compensation[sensorIndex].loadHandToSensorOffset(nh, "sensor_offset"))) {
      ROS_WARN_STREAM("Couldn't load complete ft sensor compensation data for " << ftSensorUIDs[sensorIndex] << " in " << nh.getNamespace() << ".");
    }
    ft_compensation[sensorIndex].initGravityPublisher(ftSensorUIDs[sensorIndex] + "_gravity", ftSensorUIDs[sensorIndex]);
    ee_publisher[sensorIndex] = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);
   }

  return true;
}

void ThorMangRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  // Call parent read (dealing with joint interfaces)
  DefaultRobotHWSim::readSim(time, period);

  // FT-Sensors
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
    gazebo::math::Pose sensor_pose = ft_joints_[sensorIndex]->GetWorldPose();
    gazebo::math::Matrix3 sensor_rot = sensor_pose.rot.GetAsMatrix3();
    Eigen::Matrix3d eigen_rot;
    for (unsigned int row=0; row < 3; row++) {
      for (unsigned int col=0; col < 3; col++) {
        eigen_rot(row, col) = sensor_rot[row][col];
      }
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = ftSensorUIDs[sensorIndex];
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = sensor_pose.pos.x;
    pose_msg.pose.position.y = sensor_pose.pos.y;
    pose_msg.pose.position.z = sensor_pose.pos.z;
    pose_msg.pose.orientation.x = sensor_pose.rot.x;
    pose_msg.pose.orientation.y = sensor_pose.rot.y;
    pose_msg.pose.orientation.z = sensor_pose.rot.z;
    pose_msg.pose.orientation.w = sensor_pose.rot.w;
    ee_publisher[sensorIndex].publish(pose_msg);

    ft_compensation[sensorIndex].setWorldGripperRotation(eigen_rot);
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
}

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::ThorMangRobotHWSim, gazebo_ros_control::RobotHWSim)
