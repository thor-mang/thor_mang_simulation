#ifndef THOR_MANG_FAKE_WALKING_H
#define THOR_MANG_FAKE_WALKING_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/SetModelState.h>
#include <actionlib/client/simple_action_client.h>

namespace fake_walking{



    class FakeWalking {

    public:
      FakeWalking(ros::NodeHandle& nh);
      virtual ~FakeWalking();

    protected:
      void publishRobotPose(tf::Vector3 position, float yaw );
       void setModelPose(double x, double y, double yaw);



    private:
     ros::NodeHandle nh_;
     tf::TransformBroadcaster tf_broadcaster;
      ros::ServiceClient set_model_client_;


    };
}

#endif // THOR_MANG_FAKE_WALKING_H
