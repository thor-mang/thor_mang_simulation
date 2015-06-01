#ifndef THOR_MANG_FAKE_WALKING_H
#define THOR_MANG_FAKE_WALKING_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/SetModelState.h>
#include <actionlib/client/simple_action_client.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanActionGoal.h>
#include <ros/console.h>
#include <tf/transform_listener.h>


namespace fake_walking{



class FakeWalking {

public:
    FakeWalking(ros::NodeHandle& nh);
    virtual ~FakeWalking();

protected:
    void publishRobotPose(tf::Vector3 position, float yaw );
    void setModelPose(double x, double y, double yaw);

    void executeFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionGoalConstPtr& goal);

    bool getCurrentXYYaw(double &x, double &y, double &yaw);
    void interpolateSteps(double x1, double y1, double yaw1, double x2, double y2, double yaw2);


private:
    ros::NodeHandle nh_;
    tf::TransformBroadcaster tf_broadcaster;
    ros::ServiceClient set_model_client_;
    ros::Subscriber execute_footstep_sub_;
    tf::TransformListener tf;

    const double MAX_STEP_SIZE;


};
}

#endif // THOR_MANG_FAKE_WALKING_H
