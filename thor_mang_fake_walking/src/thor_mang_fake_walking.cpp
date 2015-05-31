
#include <thor_mang_fake_walking/thor_mang_fake_walking.h>

namespace fake_walking{
    FakeWalking::FakeWalking(ros::NodeHandle &nh)
    {
       nh_ = nh;
       set_model_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

       setModelPose(0.0, 0.0, 0.0);
      ros::Rate rate(ros::Duration(2.0));
      rate.sleep();
       setModelPose(0.2, 0, 0);

    }


    FakeWalking::~FakeWalking()
    {}


    void FakeWalking::setModelPose(double x, double y, double yaw){
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
            pose.position.z = 0.86;
            msg.request.model_state.pose = pose;

            msg.request.model_state.reference_frame = "";

            tf::Vector3 vec(x,y,0.0);

            publishRobotPose(vec, yaw );



    }

    void FakeWalking::publishRobotPose(tf::Vector3 position, float yaw ){
          tf::Transform transform;
          transform.setOrigin( position);
          tf::Quaternion q;
          q.setRPY(0, 0, yaw);
          transform.setRotation(q);
          tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pelvis"));
    }






}

