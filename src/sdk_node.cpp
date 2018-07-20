
#include "std_msgs/String.h"

#include "sdk_node.h"

namespace Numurus
{

SDKNode::SDKNode(const std::string my_name) :
	initialized{false},
	n_priv{"~"}, // Create a private namespace for this node 
	name{my_name}
{}

SDKNode::~SDKNode()
{}

void SDKNode::init()
{
	// First, subscribe to the generic param messages as long as this is the first entry into this function
	if (false == initialized)
	{
		// TODO: Should we do these as services? Would be better to report success/failure to caller, but 
		// would have to have unique srv names (ROS doesn't allow multiple nodes to provide the same service)
		// and awkward for service clients.
		
		// These versions are in the public namespace so that we can support param reinit and update
		// messages to ALL of the SDK nodes simultaneously
		subscribers.push_back(n.subscribe("save_config", 3, &SDKNode::saveCfgHandler, this));

		// These versions are in this nodes private namespace so that just this node can be reinit'd and/or updated
		subscribers.push_back(n_priv.subscribe("save_config", 3, &SDKNode::saveCfgHandler, this));

		// Advertise the save_cfg coordination topics
		_update_cfg_pending_pub = n.advertise<std_msgs::String>("update_cfg_pending", 5);
		_update_cfg_complete_pub = n.advertise<std_msgs::String>("update_cfg_complete", 5);
	}

	initParams();
}

void SDKNode::initParams()
{
	// TODO: Generic param init scheme. Note, FPGA register-linked params are handled in ZynqSDKNode::initParams(), which
	// calls this method.
	initialized = true;
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

bool SDKNode::ready()
{
	for (auto &s : subscribers)
	{
		if (!s)
		{
			ROS_ERROR("%s: Invalid subscriber", name.c_str());
			initialized = false;
		}
	}
	for (auto &s : servicers)
	{
		if (!s)
		{
			ROS_ERROR("%s: Invalid servicer", name.c_str());
			initialized = false;
		}
	}
	return initialized;
}

} // namespace Numurus