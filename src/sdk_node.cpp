
#include "sdk_node.h"

namespace Numurus
{

SDKNode::SDKNode(const std::string my_name) :
	n_priv{"~"}, // Create a private namespace for this node 
	name{my_name},
	initialized{false}
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
		subscribers.push_back(n.subscribe("reinit_params", 1, &SDKNode::reinitParams, this));
		subscribers.push_back(n.subscribe("update_params", 1, &SDKNode::updateParams, this));

		// These versions are in this nodes private namespace so that just this node can be reinit'd and/or updated
		subscribers.push_back(n_priv.subscribe("reinit_params", 1, &SDKNode::reinitParams, this));
		subscribers.push_back(n_priv.subscribe("update_params", 1, &SDKNode::updateParams, this));

		initialized = true;
	}

	initParams();
}

void SDKNode::initParams()
{
	// TODO: Generic param init scheme. Note, FPGA register-linked params are handled in ZynqSDKNode::initParams(), which
	// calls this method.
}

void SDKNode::reinitParams(const std_msgs::Empty::ConstPtr& empty)
{
	ROS_INFO("%s: Reinitializing param vals. from parameter server", name.c_str());
	initParams();
}

void SDKNode::updateParams(const std_msgs::Empty::ConstPtr& empty) 
{
	ROS_INFO("%s: Updating parameter server with current param vals", name.c_str());
	// TODO: Generic param update scheme. Note, FPGA register-linked params are handled in ZynqSDKNode::updateParams(), which
	// calls this method.
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