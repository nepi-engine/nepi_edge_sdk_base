#include <vector>
#include <boost/algorithm/string.hpp>

#include "std_msgs/String.h"
#include "num_sdk_base/FileReset.h"

#include "sdk_node.h"

#define BOOL_TO_ENABLED(x)	((x)==true)? "enabled" : "disabled"

namespace Numurus
{
static std::string extractDeviceNamespace()
{
	std::vector<std::string> ns_tokens = SDKNode::splitNamespace();
	if (ns_tokens.size() < 5)
	{
		ROS_ERROR("Invalid namespace (%s) for %s", ros::this_node::getNamespace().c_str(), ros::this_node::getName().c_str());
		return "";		
	}
	// Trial and error determined these token indices
	const std::string global_ns = "/" + ns_tokens[2] + "/" + ns_tokens[3] + "/" + ns_tokens[4];	
	return global_ns;
}

static std::string extractNodeName()
{
	std::vector<std::string> ns_tokens = SDKNode::splitNamespace();
	const size_t token_count = ns_tokens.size();
	return ns_tokens[token_count - 1];
}

SDKNode::SDKNode() :
	n{extractDeviceNamespace()},
	n_priv{"~"}, // Create a private namespace for this node - just use the fully qualified node name
	_display_name{"display_name", extractNodeName(), this}, // Default to the fixed node name
	_node_name{extractNodeName()}
{
	std::vector<std::string> ns_tokens = splitNamespace();
	const size_t token_count = ns_tokens.size();
	if (token_count < 5)
	{
		ROS_ERROR("Invalid namespace (%s) for %s", ros::this_node::getNamespace().c_str(), getName().c_str());
		return;
	}
	// Trial and error determined these token indices - not sure why the first two tokens are blank strings
	device_type = ns_tokens[3];
	device_sn = ns_tokens[4];
	
	_node_name = extractNodeName();	
}

SDKNode::~SDKNode()
{}

void SDKNode::run() 
{
	init();
	ros::spin();
}

std::vector<std::string> SDKNode::splitNamespace()
{
	const std::string ns_string = ros::this_node::getName();
	std::vector<std::string> tokens;

	// Use a lambda function to provide the delimiter identifier
	boost::split(tokens, ns_string, [](char c){return c == '/';});
	return tokens;
}

void SDKNode::initPublishers()
{
	// Advertise the save_cfg coordination topics
	_store_params_pub = n.advertise<std_msgs::String>("store_params", 5);
}

void SDKNode::retrieveParams()
{
	// To appease the rosparam API, all nodes should have at least one parameter (to ensure a non-empty namespace
	// when dumping the param server contents to various config files). We use the displayName for that purpose,
	// though it is not modifiable at this generic SDKNode level
	_display_name.retrieve();
}

void SDKNode::initSubscribers()
{
	// These versions are in the public namespace so that we can support param reinit and update
	// messages to ALL of the SDK nodes simultaneously
	subscribers.push_back(n.subscribe("save_config", 3, &SDKNode::saveCfgHandler, this));
	subscribers.push_back(n.subscribe("save_config_rt", 3, &SDKNode::saveCfgRtHandler, this));
	subscribers.push_back(n.subscribe("reset", 3, &SDKNode::resetHandler, this));

	// These versions are in this nodes private namespace so that just this node can be reinit'd and/or updated
	subscribers.push_back(n_priv.subscribe("save_config", 3, &SDKNode::saveCfgHandler, this));
	subscribers.push_back(n_priv.subscribe("save_config_rt", 3, &SDKNode::saveCfgRtHandler, this));
	subscribers.push_back(n_priv.subscribe("reset", 3, &SDKNode::resetHandler, this));	
}

void SDKNode::initServices()
{
	// No services - just a placeholder for subclasses
}

void SDKNode::initServiceClients()
{
	// No clients - just a placeholder for subclasses
}

void SDKNode::saveCfgHandler(const std_msgs::Empty::ConstPtr &msg)
{
	ROS_DEBUG("%s: Initiating config save by request", getName().c_str());
	saveCfg();
}

void SDKNode::saveCfgRtHandler(const std_msgs::Bool::ConstPtr &msg)
{
	const bool save_data_updated = (msg->data != _save_cfg_rt);
	if (true == save_data_updated)
	{
		_save_cfg_rt = msg->data;
		ROS_DEBUG("%s realtime configuration saving is now %s", getName().c_str(), BOOL_TO_ENABLED(_save_cfg_rt));
		
		// If we're enabling RT saving, save the current configuration right now
		if (true == _save_cfg_rt)
		{
			saveCfg();
		}
	}
}

void SDKNode::resetHandler(const num_sdk_base::Reset::ConstPtr &msg)
{
	const uint8_t reset_type = msg->reset_type;

	switch (reset_type)
	{
	case num_sdk_base::Reset::USER_RESET:
		ROS_INFO("%s: Executing user-level reset by request", getName().c_str());
		userReset();
		break;
	case num_sdk_base::Reset::FACTORY_RESET:
	{
		ROS_INFO("%s: Executing factory reset by request", getName().c_str());
		factoryReset();
	}	
		break;
	case num_sdk_base::Reset::SOFTWARE_RESET:
		ROS_INFO("%s: Executing software reset by request", getName().c_str());
		softwareReset();
		break;
	// No implentation for HARDWARE_RESET in this base class
	case num_sdk_base::Reset::HARDWARE_RESET:
		ROS_INFO("%s: Executing hardware reset by request", getName().c_str());
		hardwareReset();
	default:
		ROS_WARN("%s: No available hardware reset. Request ignored", getName().c_str());
		// TODO: 
	}
}

void SDKNode::userReset()
{
	ros::ServiceClient client = n.serviceClient<num_sdk_base::FileReset>("user_reset");
	num_sdk_base::FileReset srv;
	srv.request.node_name = getName();
	
	if (false == client.call(srv) || false == srv.response.success)
	{
		ROS_ERROR("%s: User reset request failed", getName().c_str());
	}
	else
	{
		// Regather params from the param server now that it's been updated by config mgr
		retrieveParams();
	}
}

void SDKNode::factoryReset()
{
	ros::ServiceClient client = n.serviceClient<num_sdk_base::FileReset>("factory_reset");
	num_sdk_base::FileReset srv;
	srv.request.node_name = getName();
	if (false == client.call(srv) || false == srv.response.success)
	{
		ROS_ERROR("%s: Factory reset request failed", getName().c_str());
	}
	else
	{
		// Regather params from the param server now that it's been updated by config mgr
		retrieveParams();
	}
}

void SDKNode::softwareReset()
{
	// First, ensure the config file is reloaded on param server before restarting... userReset
	// is the most natural way.
	userReset(); 
	
	// Simply shutdown the node... it will be automatically restarted per launch file specification
	ros::shutdown();
}

void SDKNode::hardwareReset()
{
	// This generic version simply does a software reset.
	softwareReset();
}

void SDKNode::saveCfg()
{
	ROS_INFO("%s: Saving current config", getName().c_str());

	// Inform the config_mgr so it can store the file. We don't do this directly here
	// because ROS has no C++ API for rosparam
	std_msgs::String node_name;
	node_name.data = getQualifiedName(); // config_mgr expects a fully namespaced name
	_store_params_pub.publish(node_name);
}

void SDKNode::init()
{
	// initPublishers() first to ensure that any messages published by the other inits() are valid
	initPublishers();
	retrieveParams();
	initSubscribers();
	initServices();
	initServiceClients();
}

} // namespace Numurus