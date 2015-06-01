
#include <thor_mang_fake_walking/thor_mang_fake_walking.h>

namespace fake_walking{

FakeWalking::FakeWalking(ros::NodeHandle &nh) : MAX_STEP_SIZE(0.02)
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
    double x_p, y_p, yaw_p;
    if(n < 2)
    {
        ROS_WARN("FakeWalking::executeFootstepCb not executed, need at least two steps.");
        return;
    }
    if(!getCurrentXYYaw(x_p, y_p, yaw_p))
    {
        ROS_WARN("FakeWalking::executeFootstepCb not executed, no x,y position of robot available.");
        return;
    }

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
        
        interpolateSteps(x_p, y_p, yaw_p, x, y, yaw);

        ROS_INFO("x = %f, y = %f, yaw = %f", x, y, yaw);
        setModelPose(x, y, yaw);

        x_p = x;
        y_p = y;
	yaw_p = yaw;
        // TODO: Interpolate steps until euclidean distance is less than 1cm / predefined system!
    }
}

bool FakeWalking::getCurrentXYYaw(double &x, double &y, double &yaw)
{
    tf::StampedTransform pelvis_transform;
    ros::Time now = ros::Time::now();
    try {
        tf.waitForTransform("world", "pelvis", now, ros::Duration(1.0));
        tf.lookupTransform("world", "pelvis", now, pelvis_transform);
        x = pelvis_transform.getOrigin().x();
        y = pelvis_transform.getOrigin().y();
//        yaw = pelvis_transform.get
        yaw = 0;
        return true;
    } catch (std::runtime_error& e) {
        ROS_WARN("Could not transform look_at position to target frame_id %s", e.what());
        return false;
    }
}

void FakeWalking::interpolateSteps(double x1, double y1, double yaw1, double x2, double y2, double yaw2)
{
    double d = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));
    double dy = (y2 - y1) / d;
    double dx = (x2 - x1) / d;

    int n = std::floor(d / MAX_STEP_SIZE);
    ros::Rate rate(ros::Duration(0.1));
//    double step_yaw = (yaw2 - yaw1) / n;

    for(int i = 0; i < n; i++)
    {
        double x = x1 + dx * (i + 1) * MAX_STEP_SIZE;
        double y = y1 + dy * (i + 1) * MAX_STEP_SIZE;
//        double yaw = yaw1 + (i + 1) * step_yaw;
        rate.sleep();
        setModelPose(x, y, yaw2);
    }
}


FakeWalking::~FakeWalking()
{}


void FakeWalking::setModelPose(double x, double y, double yaw){
    double z = 0.86;
    tf::StampedTransform pelvis_transform;
    ros::Time now = ros::Time::now();

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

