#include "ros/ros.h"
#include "cob_srvs/Trigger.h"
#include "ros_canopen/posmsg.h"
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummycontrollernode_single_device");
  ros::NodeHandle n;

  ros::Publisher setPosPublisher = n.advertise<ros_canopen::posmsg>("/chain1/setpos", 1000);
  ros::ServiceClient initClient = n.serviceClient<cob_srvs::Trigger>("/chain1/init");
  ros::ServiceClient homingClient = n.serviceClient<cob_srvs::Trigger>("/chain1/homing");
  ros::ServiceClient IPmodeClient = n.serviceClient<cob_srvs::Trigger>("/chain1/IPmode");
  cob_srvs::Trigger srv;

  initClient.call(srv);  // init all devices in chain1
  homingClient.call(srv); // perform homing of all devices in chain1
  IPmodeClient.call(srv); // put all devices in chain1 in IP mode

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // move device a bit in IP mode:
  ros_canopen::posmsg msg;
  uint32_t pos = 0;
  ros::Rate loop_rate(100);

  while (ros::ok()) {

    std::vector<int> mypos = {pos};
    msg.positions = mypos;

    setPosPublisher.publish(msg);
    pos += 100;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
