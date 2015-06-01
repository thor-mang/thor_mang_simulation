
#include <thor_mang_fake_walking/thor_mang_fake_walking.h>

namespace fake_walking{

FakeWalking::FakeWalking(ros::NodeHandle &nh)
{
    nh_ = nh;
    set_model_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    execute_footstep_sub_ = nh_.subscribe("/vigir/footstep_manager/execute_step_plan/goal", 10, &FakeWalking::executeFootstepCb, this);


    setModelPose(0.0, 0.0, 0.0);
   ros::Rate rate(ros::Duration(2.0));
    rate.sleep();
   setModelPose(0.0, 0, 0);
//    rate.sleep();
//    setModelPose(0.2, 0.5, 0);
//    rate.sleep();
//    setModelPose(0.2, 0.5, M_PI / 2);


    // Next: Subscribe
    //       /thor_mang/step_controller/execute_step_plan/goal
}

void FakeWalking::executeFootstepCb(const vigir_footstep_planning_msgs::ExecuteStepPlanActionGoalConstPtr& goal)
{
    ROS_INFO("FakeWalking::executeFootstepCb");

    int n = goal->goal.step_plan.steps.size();
    if(n < 2)
        ROS_WARN("FakeWalking::executeFootstepCb not executed, need at least two steps.");

    for(int i = 0; i < n; i += 2)
    {
        vigir_footstep_planning_msgs::Step p1 = goal->goal.step_plan.steps[i];
        vigir_footstep_planning_msgs::Step p2;
        if(n > i + 1)
            p2 = goal->goal.step_plan.steps[i + 1];
        else
        {
            p2 = p1;
            p1 = goal->goal.step_plan.steps[i - 1];
        }
        ros::Rate rate(ros::Duration(0.1));
        rate.sleep();

        geometry_msgs::Pose pp1 = p1.foot.pose;
        geometry_msgs::Pose pp2 = p2.foot.pose;
        double x1 = pp1.position.x, y1 = pp1.position.y, x2 = pp2.position.x, y2 = pp2.position.y;
        double x = (x1 + x2) / 2, y =  (y1 + y2) / 2;

        tf::Quaternion q(pp1.orientation.x, pp1.orientation.y, pp1.orientation.z, pp1.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        ROS_INFO("x = %f, y = %f, yaw = %f", x, y, yaw);
        setModelPose(x, y, yaw);

    }
}



FakeWalking::~FakeWalking()
{}


void FakeWalking::setModelPose(double x, double y, double yaw){
    double z = 0.86;
    tf::StampedTransform pelvis_transform;

    try {
        tf.waitForTransform("world", "pelvis", now, ros::Duration(1.0));
        tf.lookupTransform("world", "pelvis", now, pelvis_transform);
        z = pelvis_transform.getOrigin().z();
    } catch (std::runtime_error& e) {
        ROS_WARN("Could not transform look_at position to target frame_id %s", e.what());

    }

    std::cout << " setting pose " << x << "," << y << ", " << yaw << std::endl;
    set_model_client_.waitForExistence();
    gazebo_msgs::SetModelState msg;
    msg.request.model_state.model_name = "robot_description";

    geometry_msgs::Pose pose;
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.orientation.x=q.x();
    pose.orientation.y=q.y();
    pose.orientation.z=q.z();
    pose.orientation.w=q.w();

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    msg.request.model_state.pose = pose;

    msg.request.model_state.reference_frame = "";

    tf::Vector3 vec(x,y,z);

    set_model_client_.call(msg);

    publishRobotPose(vec, yaw );



}

void FakeWalking::publishRobotPose(tf::Vector3 position, float yaw ){
    tf::Transform transform;
    transform.setOrigin( position);
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    transform.setRotation(q);
    std::cout << "publishing world " << std::endl;
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pelvis"));
     std::cout << "published world " << std::endl;
}






}

