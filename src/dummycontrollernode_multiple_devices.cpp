#include "ros/ros.h"
#include "cob_srvs/Trigger.h"
#include "ros_canopen/posmsg.h"
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>

std::vector<int> current_pos = {0,0,0,0,0,0};
std::mutex m;

void getCurrentPosCallback(const ros_canopen::posmsg::ConstPtr &msg) {
  std::vector<int> positions = msg->positions;
  /* for (auto it : positions) 
    std::cout << it << "   ";
    std::cout << std::endl; */
  // m.lock();
  current_pos = positions;

  /*  for (auto it : current_pos) 
    std::cout << it << "   ";
    std::cout << std::endl; */

  // m.unlock();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummycontrollernode_multiple_devices");
  ros::NodeHandle n;

  ros::Publisher setPosPublisher = n.advertise<ros_canopen::posmsg>("/arm1/setpos", 1000);
  ros::Subscriber getCurrentPosSubscriber = 
    n.subscribe<ros_canopen::posmsg>("/arm1/getcurrentpos", 100, getCurrentPosCallback);
  ros::ServiceClient initClient = n.serviceClient<cob_srvs::Trigger>("/arm1/init");
  ros::ServiceClient homingClient = n.serviceClient<cob_srvs::Trigger>("/arm1/homing");
  ros::ServiceClient IPmodeClient = n.serviceClient<cob_srvs::Trigger>("/arm1/IPmode");
  cob_srvs::Trigger srv;

  initClient.call(srv);  // init all devices in arm1
  homingClient.call(srv); // perform homing of all devices in arm1
  IPmodeClient.call(srv); // put all devices in arm1 in IP mode

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // std::vector<int> current_pos = canopen::getCurrentPosCallback("arm1");
  // std::vector<int> request_pos = current_pos;
  

  // move arm1 a bit in IP mode:
  ros_canopen::posmsg msg;
  uint32_t pos = 0;
  std::vector<int> incr = {20,20,20,20,20,20};
  std::vector<int> request_pos = current_pos;
  // std::vector<int> rr;
  ros::Rate loop_rate(100);
  std::cout << "hi" << std::endl;
  while (ros::ok()) {
    std::cout << "ho" << std::endl;
    
    // rr = current_pos;
    /*for (int k=0; k<current_pos.size(); k++)
      std::cout << current_pos[k] << "  ";
      std::cout << std::endl; */

    for (int k=0; k<6; k++)  {
      if  (current_pos[k] <= -300 || current_pos[k] > 7000) {
	incr[k] = 20;
      } else  {
	incr[k] = -20;
      } 
    
      if (abs(request_pos[k] - current_pos[k]) > 200) {
	request_pos[k] = request_pos[k] + incr[k]/2;
      } else {
	request_pos[k] = request_pos[k] + incr[k];
      } 
    }

    /* std::vector<int> mypos = {pos};
    msg.positions = mypos;

    setPosPublisher.publish(msg);
    pos += 100; */
    /* std::cout << "pos: ";
    for (auto it : current_pos)
      std::cout << it << "  ";
      std::cout << std::endl; */
    msg.positions = request_pos;
    setPosPublisher.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
