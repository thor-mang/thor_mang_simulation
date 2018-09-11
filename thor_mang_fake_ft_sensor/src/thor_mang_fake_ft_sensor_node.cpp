#include <thor_mang_fake_ft_sensor/thor_mang_fake_ft_sensor_node.h>

namespace thormang3
{
FakeFTSensorNode::FakeFTSensorNode(ros::NodeHandle& nh)
  : fake_ft_sensor_(nh)
{
  fake_ft_sensor_.initialize("robot_description");

  // schedule main update loop
  update_timer_ = nh.createTimer(nh.param("control_rate", 100.0), &FakeFTSensorNode::update, this);
}

FakeFTSensorNode::~FakeFTSensorNode()
{
}

void FakeFTSensorNode::update(const ros::TimerEvent &event)
{
  if (!event.last_real.isZero())
    fake_ft_sensor_.process();
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thor_mang_fake_ft_sensors");

  ros::NodeHandle nh;

  // ensure that node's services are set up in proper namespace
  if (nh.getNamespace().size() <= 1)
    nh = ros::NodeHandle("~");

  thormang3::FakeFTSensorNode node(nh);

  ros::spin();

  return 0;
}
