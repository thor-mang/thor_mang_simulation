//=================================================================================================
// Copyright (c) 2018, Felix Sternkopf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
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

#ifndef THOR_MANG_FAKE_FT_SENSOR_H__
#define THOR_MANG_FAKE_FT_SENSOR_H__

#include <ros/ros.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <boost/shared_ptr.hpp>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

namespace thormang3
{

class FakeFTSensor
{
public:
  FakeFTSensor();
  FakeFTSensor(ros::NodeHandle& nh);
  ~FakeFTSensor();

  bool initialize(std::string robotParamName);
  void process();
  void reset();

protected:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states);

private:
  KDL::Tree robot_;
  ros::NodeHandle nh_;

  ros::Subscriber jointStateSub_;

  ros::Publisher leftFTSensorPub_;
  ros::Publisher rightFTSensorPub_;
};
}

#endif
