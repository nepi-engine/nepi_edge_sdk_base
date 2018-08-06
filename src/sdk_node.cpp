
#include "std_msgs/String.h"

#include "sdk_node.h"

namespace Numurus
{

SDKNode::SDKNode(const std::string my_name) :
	n_priv{"~"}, // Create a private namespace for this node 
	name{my_name}
{}

SDKNode::~SDKNode()
{}

void SDKNode::run() 
{
	init();
	ros::spin();
}

void SDKNode::initPublishers()
{
	// Advertise the save_cfg coordination topics
	_update_cfg_pending_pub = n.advertise<std_msgs::String>("update_cfg_pending", 5);
	_update_cfg_complete_pub = n.advertise<std_msgs::String>("update_cfg_complete", 5);	
}

void SDKNode::initParams()
{
	// Just a placeholder for subclasses
}

void SDKNode::initSubscribers()
{
	// These versions are in the public namespace so that we can support param reinit and update
	// messages to ALL of the SDK nodes simultaneously
	subscribers.push_back(n.subscribe("save_config", 3, &SDKNode::saveCfgHandler, this));

	// These versions are in this nodes private namespace so that just this node can be reinit'd and/or updated
	subscribers.push_back(n_priv.subscribe("save_config", 3, &SDKNode::saveCfgHandler, this));	
}

void SDKNode::initServices()
{
	// No services - just a placeholder for subclasses
}

void SDKNode::updateParams()
{
	// No params (yet) - just a placeholder for subclasses
}

void SDKNode::saveCfgHandler(const std_msgs::Empty::ConstPtr &msg)
{
	ROS_DEBUG("%s: Initiating config save by request", name.c_str());
	saveCfg();
}

void SDKNode::saveCfg()
{
	ROS_INFO("%s: Saving current config", name.c_str());

	// First, inform that we'll be updating the param server
	std_msgs::String name;
	name.data = ros::this_node::getName();
	_update_cfg_pending_pub.publish(name);

	// Now update it
	updateParams();

	// Now inform complete
	_update_cfg_complete_pub.publish(name);
}

void SDKNode::init()
{
	// initPublishers() first to ensure that any messages published by the other inits() are valid
	initPublishers();
	initParams();
	initSubscribers();
	initServices();
}

} // namespace Numurus