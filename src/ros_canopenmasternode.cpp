#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cob_srvs/Trigger.h"
#include "ros_canopen/posmsg.h"
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <canopenmaster.h>
#include <XmlRpcValue.h>

typedef boost::function<bool(cob_srvs::Trigger::Request&, cob_srvs::Trigger::Response&)> TriggerCallbackType;
typedef boost::function<void(const ros_canopen::posmsg&)> setPosCallbackType;

bool initChainCallback(cob_srvs::Trigger::Request &req,
		       cob_srvs::Trigger::Response &res, std::string chainName) {
  canopen::initCallback(chainName);
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

bool IPmodeChainCallback(cob_srvs::Trigger::Request &req,
			  cob_srvs::Trigger::Response &res, std::string chainName) {
  canopen::IPmodeCallback(chainName);
  res.success.data = true;
  res.error_message.data = "";
  return true;
}

void setPosChainCallback(const ros_canopen::posmsg &msg, std::string chainName) {
  std::vector<int> positions = msg.positions;
  canopen::setPosCallback(chainName, positions);
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

  XmlRpc::XmlRpcValue xmlrpc_array;
  bool success = n.getParam("/chaindesc", xmlrpc_array);
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
  std::vector<setPosCallbackType> setPosCallbacks;
  std::vector<ros::Subscriber> setPosSubscribers;
  std::map<std::string, ros::Publisher> getCurrentPosPublishers;
  for (auto it : canopen::chainMap) {
    initCallbacks.push_back( boost::bind(initChainCallback, _1, _2, it.first) );
    initServices.push_back( n.advertiseService(it.first + "/init", initCallbacks.back()) );
    homingCallbacks.push_back( boost::bind(homingChainCallback, _1, _2, it.first) );
    homingServices.push_back( n.advertiseService(it.first + "/homing", homingCallbacks.back()) );
    IPmodeCallbacks.push_back( boost::bind(IPmodeChainCallback, _1, _2, it.first) );
    IPmodeServices.push_back( n.advertiseService(it.first + "/IPmode", IPmodeCallbacks.back()) );
    setPosCallbacks.push_back( boost::bind(setPosChainCallback, _1, it.first) );
    setPosSubscribers.push_back( n.subscribe<ros_canopen::posmsg>(it.first + "/setpos", 100, setPosCallbacks.back()) );
    getCurrentPosPublishers[it.first] = n.advertise<ros_canopen::posmsg>(it.first + "/getcurrentpos", 1000);
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
  ros::Rate loop_rate(50);
  // publish on the advertised topics; separate topics for each chain (so far, only getcurrentpos):
  while (ros::ok()) {
    
    for (auto it : getCurrentPosPublishers) {
      std::vector<int> currentPos = canopen::getCurrentPosCallback(it.first);
      msg.positions = currentPos;
      it.second.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
