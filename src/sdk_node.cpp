
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
	_store_params_pub = n.advertise<std_msgs::String>("store_params", 5);
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

void SDKNode::resetHandler(const num_sdk_base::Reset::ConstPtr &msg)
{
	const uint8_t reset_type = msg->reset_type;

	switch (reset_type)
	{
	case num_sdk_base::Reset::USER_RESET:
		// Just re-gather the params from the param server
		ROS_INFO("%s: Executing user-level reset by request", name.c_str());
		initParams();
		break;
	case num_sdk_base::Reset::FACTORY_RESET:
	{
		ROS_INFO("%s: Executing factory reset by request", name.c_str());
		ros::ServiceClient client = n.serviceClient<num_sdk_base::FactoryReset>("factory_reset");
		num_sdk_base::FactoryReset srv;
		srv.request.node_name = name;
		if (false == client.call(srv) || false == srv.response.success)
		{
			ROS_ERROR("%s: Factory reset request failed", name.c_str());
		}
		else
		{
			// Regather params from the param server now that it's been updated by config mgr
			initParams();
		}
	}	
		break;
	case num_sdk_base::Reset::SOFTWARE_RESET:
		ROS_INFO("%s: Executing software reset by request", name.c_str());
		// TODO: Terminate the node
		break;
	// No implentation for HARDWARE_RESET in this base class
	//case Reset::HARDWARE_RESET:
	default:
		ROS_WARN("%s: No available hardware reset. Request ignored", name.c_str());
		// TODO: 
	}
}

void SDKNode::saveCfg()
{
	ROS_INFO("%s: Saving current config", name.c_str());

	// First, update the param server
	updateParams();

	// Now, inform the config_mgr so it can store the file. We don't do this directly here
	// because ROS has no C++ API for rosparam
	std_msgs::String node_name;
	node_name.data = name;
	_store_params_pub.publish(node_name);
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