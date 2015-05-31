#ifndef THOR_MANG_FAKE_WALKING_H
#define THOR_MANG_FAKE_WALKING_H

#include <ros/ros.h>

namespace fake_walking{



    class FakeWalking {

    public:
      FakeWalking(ros::NodeHandle& nh);
      virtual ~FakeWalking();

    protected:



    private:
     ros::NodeHandle nh_;


    };
}

#endif // THOR_MANG_FAKE_WALKING_H
