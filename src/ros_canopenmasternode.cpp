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
typedef boost::function<
  bool(cob_srvs::SetOperationMode::Request&,
       cob_srvs::SetOperationMode::Response&)> SetOperationModeCallbackType;

bool CANopenInit(cob_srvs::Trigger::Request &req,
		 cob_srvs::Trigger::Response &res, std::string chainName) {
  canopen::chainMap[chainName]->CANopenInit();
  res.success.data = true;
  res.error_message.data = "";
  return true;
}

bool setOperationModeCallback(cob_srvs::SetOperationMode::Request &req,
			      cob_srvs::SetOperationMode::Response &res, std::string chainName) {
  res.success.data = true;  // for now this service is just a dummy, not used elsewhere
  // res.error_message.data = "";
  return true;
}

void setVel(const brics_actuator::JointVelocities &msg, std::string chainName) {
  std::vector<double> velocities;
  for (auto it : msg.velocities)
    velocities.push_back( it.value); 
  canopen::chainMap[chainName]->setVel(velocities); 
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
  ros::NodeHandle n("~");

  canopen::using_master_thread = true;
  canopen::syncInterval = std::chrono::milliseconds(10); // todo: parse from config file

  XmlRpc::XmlRpcValue xmlrpc_array;
  if (! n.getParam("/chaindesc", xmlrpc_array)) {
  // if (! n.getParam("chains", xmlrpc_array)) {
    std::cout << "No chain description on parameter server; aborting" << std::endl;
    return -1;
  }
  auto chainDesc = parseChainDescription(xmlrpc_array);
  canopen::initChainMap(chainDesc);

  std::cout << "hi" << std::endl;

  // set up services, subscribers, and publishers for each of the chains:
  std::vector<TriggerType> initCallbacks;
  std::vector<ros::ServiceServer> initServices;
  std::vector<SetOperationModeCallbackType> setOperationModeCallbacks;
  std::vector<ros::ServiceServer> setOperationModeServices;

  std::vector<JointVelocitiesType> jointVelocitiesCallbacks;
  std::vector<ros::Subscriber> jointVelocitiesSubscribers;
  std::map<std::string, ros::Publisher> currentOperationModePublishers;
  std::map<std::string, ros::Publisher> statePublishers;
  ros::Publisher jointStatesPublisher = 
    n.advertise<sensor_msgs::JointState>("/joint_states", 100);
  
  for (auto it : canopen::chainMap) {
    initCallbacks.push_back( boost::bind(CANopenInit, _1, _2, it.first) );
    initServices.push_back
      (n.advertiseService("/" + it.first + "/init", initCallbacks.back()) );
    setOperationModeCallbacks.push_back( boost::bind(setOperationModeCallback, _1, _2, it.first) );
    setOperationModeServices.push_back( n.advertiseService("/" + it.first + "/set_operation_mode", setOperationModeCallbacks.back()) );

    jointVelocitiesCallbacks.push_back( boost::bind(setVel, _1, it.first) );
    jointVelocitiesSubscribers.push_back
      (n.subscribe<brics_actuator::JointVelocities>
       ("/" + it.first + "/command_vel", 100, jointVelocitiesCallbacks.back())  );
    // todo: remove global namespace; ticket

    currentOperationModePublishers[it.first] =
      n.advertise<std_msgs::String>
      ("/" + it.first + "/current_operationmode", 1000);
    
    statePublishers[it.first] =
      n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>
      ("/" + it.first + "/state", 1000);
  }

  if (!canopen::openConnection("/dev/pcan32")) {
    std::cout << "Cannot open CAN device; aborting." << std::endl;
    return -1;
  } 
  canopen::initListenerThread();
  canopen::initIncomingPDOProcessorThread();
  canopen::initMasterThread();
  canopen::initNMT();
  for (auto it : canopen::chainMap) 
    it.second->CANopenInit();

  double lr = 1000.0 / std::chrono::duration_cast
    <std::chrono::milliseconds>(canopen::syncInterval).count();
  std::cout << "Loop rate: " << lr << std::endl;
  ros::Rate loop_rate(lr); 

  canopen::ChainState cs;

  while (ros::ok()) {
    
    for (auto it : canopen::chainMap) { // iterate over all chains, get current pos and vel and publish as topics:
      cs = it.second->getChainState();

      sensor_msgs::JointState js;  
      // std::vector<std::string> ss = {"tray_1_joint", "tray_2_joint", "tray_3_joint"};
      std::vector<std::string> ss = {"arm_1_joint", "arm_2_joint", "arm_3_joint","arm_4_joint", "arm_5_joint", "arm_6_joint"};
      js.name = ss;
      js.header.stamp = ros::Time::now(); // todo: should be timestamp of hardware msg
      // hack:
      //for (int i=0; i<cs.actualPos.size(); i++) {
      //	cs.actualPos[i] = - cs.actualPos[i];
      //	cs.actualVel[i] = - cs.actualVel[i];
      //	cs.desiredPos[i] = - cs.desiredPos[i];
      //	cs.desiredVel[i] = - cs.desiredVel[i];
      //      } // end of hack
      js.position = cs.actualPos;
      js.velocity = cs.actualVel;
      js.effort = {0,0,0,0,0,0};
      jointStatesPublisher.publish(js);

      pr2_controllers_msgs::JointTrajectoryControllerState jtcs; // todo: check if this is correct
      jtcs.header.stamp = js.header.stamp;
      jtcs.actual.positions = cs.actualPos;
      jtcs.actual.velocities = cs.actualVel;
      jtcs.desired.positions = cs.desiredPos;
      jtcs.desired.velocities = cs.desiredVel;
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

