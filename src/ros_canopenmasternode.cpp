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
       cob_srvs::Trigger::Response&)> TriggerCallbackType;
typedef boost::function<
  bool(cob_srvs::SetOperationMode::Request&,
       cob_srvs::SetOperationMode::Response&)> SetOperationModeCallbackType;
typedef boost::function<
  void(const ros_canopen::posmsg&)> setPosCallbackType;
typedef boost::function<
  void(const brics_actuator::JointVelocities&)> JointVelocitiesCallbackType;

bool initChainCallback(cob_srvs::Trigger::Request &req,
		       cob_srvs::Trigger::Response &res, std::string chainName) {
  canopen::initCallback(chainName, canopen::sync_deltaT_msec);
  res.success.data = true;
  res.error_message.data = "";
  return true;
}

bool homingChainCallback(cob_srvs::Trigger::Request &req,
			  cob_srvs::Trigger::Response &res, std::string chainName) {
  canopen::homingCallback(chainName);
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


bool IPmodeChainCallback(cob_srvs::Trigger::Request &req,
			  cob_srvs::Trigger::Response &res, std::string chainName) {
  canopen::IPmodeCallback(chainName);
  res.success.data = true;
  res.error_message.data = "";
  return true;
}

/* void setPosChainCallback(const ros_canopen::posmsg &msg, std::string chainName) {
  std::vector<double> positions = msg.positions;
  canopen::setPosCallback(chainName, positions);
  }*/

void jointVelocitiesCallback(const brics_actuator::JointVelocities &msg, std::string chainName) {
  std::vector<double> velocities;
  for (auto it : msg.velocities)
    velocities.push_back(it.value);
 
  std::cout << "Velocities: ";
  for (auto v : velocities)
    std::cout << v << "  ";
  std::cout << std::endl;

  canopen::setVelCallback(chainName, velocities);
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

  int sync_deltaT_msec_int = 20; // todo!
  canopen::sync_deltaT_msec = std::chrono::milliseconds(sync_deltaT_msec_int);
  std::cout << "sync rate: " << sync_deltaT_msec_int << std::endl;


  XmlRpc::XmlRpcValue xmlrpc_array;
  bool success = n.getParam("/chaindesc", xmlrpc_array); // todo: not needed?
  auto chainDesc = parseChainDescription(xmlrpc_array);
  canopen::initChainMap(chainDesc);

  // read in chain description file given as command line argument, e.g.
  // /home/tys/git/other/canopen/demo/single_device.csv
  // /home/tys/git/other/canopen/demo/multiple_devices.csv
  /*  if (argc!=2 || !exists(  boost::filesystem::path(argv[1]) )) {
    std::cout << "File not found. Please provide a description file as command line argument" << std::endl;
    return -1;
    }*/
  // canopen::initChainMap(argv[1]);  todo!

  // set up services, subsribers, and publishers for each of the chains:
  std::vector<TriggerCallbackType> initCallbacks;
  std::vector<ros::ServiceServer> initServices;
  std::vector<TriggerCallbackType> homingCallbacks;
  std::vector<ros::ServiceServer> homingServices;
  std::vector<TriggerCallbackType> IPmodeCallbacks;
  std::vector<ros::ServiceServer> IPmodeServices;
  std::vector<SetOperationModeCallbackType> setOperationModeCallbacks;
  std::vector<ros::ServiceServer> setOperationModeServices;

  // std::vector<setPosCallbackType> setPosCallbacks;
  // std::vector<ros::Subscriber> setPosSubscribers;

  std::vector<JointVelocitiesCallbackType> jointVelocitiesCallbacks;
  std::vector<ros::Subscriber> jointVelocitiesSubscribers;

  std::map<std::string, ros::Publisher> currentOperationModePublishers;
  // std::map<std::string, ros::Publisher> getCurrentPosPublishers;
  std::map<std::string, ros::Publisher> statePublishers;
  ros::Publisher jointStatesPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 100);
  for (auto it : canopen::chainMap) {
    initCallbacks.push_back( boost::bind(initChainCallback, _1, _2, it.first) );
    initServices.push_back( n.advertiseService(it.first + "/init", initCallbacks.back()) );
    homingCallbacks.push_back( boost::bind(homingChainCallback, _1, _2, it.first) );
    homingServices.push_back( n.advertiseService(it.first + "/homing", homingCallbacks.back()) );
    IPmodeCallbacks.push_back( boost::bind(IPmodeChainCallback, _1, _2, it.first) );
    IPmodeServices.push_back( n.advertiseService(it.first + "/IPmode", IPmodeCallbacks.back()) );
    setOperationModeCallbacks.push_back( boost::bind(setOperationModeCallback, _1, _2, it.first) );
    setOperationModeServices.push_back( n.advertiseService(it.first + "/set_operation_mode", setOperationModeCallbacks.back()) );


    // setPosCallbacks.push_back( boost::bind(setPosChainCallback, _1, it.first) );
    // setPosSubscribers.push_back( n.subscribe<ros_canopen::posmsg>(it.first + "/setpos", 100, setPosCallbacks.back()) );

    jointVelocitiesCallbacks.push_back( boost::bind(jointVelocitiesCallback, _1, it.first) );
    jointVelocitiesSubscribers.push_back( n.subscribe<brics_actuator::JointVelocities>(it.first + "/command_vel", 100, jointVelocitiesCallbacks.back()) );

    currentOperationModePublishers[it.first] = n.advertise<std_msgs::String>(it.first + "/current_operationmode", 1000);
    // getCurrentPosPublishers[it.first] = n.advertise<ros_canopen::posmsg>(it.first + "/getcurrentpos", 1000);
    statePublishers[it.first] = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(it.first + "/state", 1000);

  }

  // initialize CAN device driver:
  if (!canopen::openConnection("/dev/pcan32")) {
    std::cout << "Cannot open CAN device; aborting." << std::endl;
    return -1;
  } 
  canopen::initNMT();

  canopen::initListenerThread();
  canopen::initIncomingPDOProcessorThread();
  canopen::initMasterThread();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  ros_canopen::posmsg msg;
  //int sync_deltaT_msec_int = static_cast<int>(canopen::sync_deltaT_msec.count());
  //std::cout << "ROS loop rate: " << sync_deltaT_msec_int << std::endl;
  double lr = 1000.0 / static_cast<double>(sync_deltaT_msec_int);
  std::cout << "Loop rate: " << lr << std::endl;
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
      js.name = ss;
      js.header.stamp = ros::Time::now(); // todo: should be timestamp of hardware msg
      js.position = actualPos;
      js.velocity = actualVel;
      js.effort = {0,0,0,0,0,0};
      jointStatesPublisher.publish(js);

      pr2_controllers_msgs::JointTrajectoryControllerState jtcs;
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
