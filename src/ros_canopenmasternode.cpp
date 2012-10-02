#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "brics_actuator/JointVelocities.h"
#include "cob_srvs/Trigger.h"
#include "cob_srvs/SetOperationMode.h"
#include "ros_canopen/posmsg.h"
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <canopenmaster.h>
#include <XmlRpcValue.h>

typedef boost::function<
  bool(cob_srvs::Trigger::Request&,
       cob_srvs::Trigger::Response&)> TriggerType;
typedef boost::function<
  void(const brics_actuator::JointVelocities&)> JointVelocitiesType;

bool CANopenInit(cob_srvs::Trigger::Request &req,
		 cob_srvs::Trigger::Response &res, std::string chainName) {
  canopen::chainMap[chainName]->CANopenInit();
  res.success.data = true;
  res.error_message.data = "";
  return true;
}

void setVel(const brics_actuator::JointVelocities &msg, std::string chainName) {
  std::vector<double> velocities;
  for (auto it : msg.velocities)
    velocities.push_back(it.value);
  canopen::chainMap[chainName]->setVel(velocities); // { velocities[0] } hack
}

std::vector<canopen::ChainDescription> parseChainDescription(XmlRpc::XmlRpcValue xx) {
  std::vector<canopen::ChainDescription> chainDesc;
  
  for (int i=0; i<xx.size(); i++) {
    canopen::ChainDescription chain;
    chain.name = static_cast<std::string>(xx[i]["name"]);
    auto yy = xx[i]["devices"];
    for (int j=0; j<yy.size(); j++) {
      canopen::DeviceDescription device;
      device.name = static_cast<std::string>(yy[j]["name"]);
      device.id = static_cast<int>(yy[j]["id"]);
      device.bus = static_cast<std::string>(yy[j]["bus"]);
      chain.devices.push_back(device);
    }
    chainDesc.push_back(chain);
  }
  return chainDesc;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_canopenmasternode");
  ros::NodeHandle n;

  canopen::using_master_thread = true;
  canopen::syncInterval = std::chrono::milliseconds(20); // todo: parse from config file

  XmlRpc::XmlRpcValue xmlrpc_array;
  if (! n.getParam("/chaindesc", xmlrpc_array)) {
    std::cout << "No chain description on parameter server; aborting" << std::endl;
    return -1;
  }
  auto chainDesc = parseChainDescription(xmlrpc_array);
  canopen::initChainMap(chainDesc);

  std::cout << "hi" << std::endl;

  // set up services, subscribers, and publishers for each of the chains:
  std::vector<TriggerType> initCallbacks;
  std::vector<ros::ServiceServer> initServices;
  std::vector<JointVelocitiesType> jointVelocitiesCallbacks;
  std::vector<ros::Subscriber> jointVelocitiesSubscribers;
  std::map<std::string, ros::Publisher> currentOperationModePublishers;
  std::map<std::string, ros::Publisher> statePublishers;
  ros::Publisher jointStatesPublisher = 
    n.advertise<sensor_msgs::JointState>("/joint_states", 100);
  
  for (auto it : canopen::chainMap) {
    initCallbacks.push_back( boost::bind(CANopenInit, _1, _2, it.first) );
    initServices.push_back
      (n.advertiseService(it.first + "/init", initCallbacks.back()) );
    jointVelocitiesCallbacks.push_back( boost::bind(setVel, _1, it.first) );
    jointVelocitiesSubscribers.push_back
      (n.subscribe<brics_actuator::JointVelocities>
       (it.first + "/command_vel", 100, jointVelocitiesCallbacks.back())  );

    currentOperationModePublishers[it.first] =
      n.advertise<std_msgs::String>
      (it.first + "/current_operationmode", 1000);
    
    statePublishers[it.first] =
      n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>
      (it.first + "/state", 1000);
  }

  if (!canopen::openConnection("/dev/pcan32")) {
    std::cout << "Cannot open CAN device; aborting." << std::endl;
    return -1;
  } 
  canopen::initListenerThread();
  canopen::initIncomingPDOProcessorThread();
  canopen::initMasterThread();
  canopen::initNMT();

  double lr = 1000.0 / std::chrono::duration_cast
    <std::chrono::milliseconds>(canopen::syncInterval).count();
  std::cout << "Loop rate: " << lr << std::endl;

  while (true) {}
  return 0;
}
  /*
  ros::Rate loop_rate(lr);  // todo! now correct (?)
  // publish on the advertised topics:
  while (ros::ok()) {
    
    for (auto it : canopen::chainMap) { // iterate over all chains, get current pos and vel and publish as topics:
      std::vector<double> actualPos = canopen::getActualPosCallback(it.first);
      std::vector<double> actualVel = canopen::getActualVelCallback(it.first);
      std::vector<double> desiredPos = canopen::getDesiredPosCallback(it.first);
      std::vector<double> desiredVel = canopen::getDesiredVelCallback(it.first);
      
      // msg.positions = actualPos;
      // it.second.publish(msg);
      
      // todo: before info from hardware received, do not publish joitn states and operation mode!!
      // donÄt publish anythign before

      // todo: this one not needed for controller (?); publishing only dummy info for now
      // pro chain, weil verscxhiedene frequenzen pro chain
      sensor_msgs::JointState js;  
      std::vector<std::string> ss = {"arm_1_joint"}; // ,"arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint"}; // todo!
      //      std::vector<std::string> ss = {"arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint"}; // todo!
      js.name = ss;
      js.header.stamp = ros::Time::now(); // todo: should be timestamp of hardware msg
      js.position = actualPos;
      js.velocity = actualVel;
      js.effort = {0,0,0,0,0,0};
      jointStatesPublisher.publish(js);

      pr2_controllers_msgs::JointTrajectoryControllerState jtcs; // todo: not sure this is correct
      jtcs.header.stamp = js.header.stamp;
      jtcs.actual.positions = actualPos;
      jtcs.actual.velocities = actualVel;
      jtcs.desired.positions = desiredPos;
      jtcs.desired.velocities = desiredVel;
      statePublishers[it.first].publish(jtcs);
      
      std_msgs::String opmode;
      opmode.data = "velocity";
      currentOperationModePublishers[it.first].publish(opmode);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
*/
