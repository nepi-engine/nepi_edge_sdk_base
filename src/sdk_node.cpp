
#include "sdk_node.h"
#include "num_common.h"

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
	initRegisterParams();
	// TODO: Non-register params
}

void SDKNode::reinitParams(const std_msgs::Empty::ConstPtr& empty)
{
	ROS_INFO("%s: Reinitializing param vals. from parameter server", name.c_str());
	initParams();
}

void SDKNode::updateParams(const std_msgs::Empty::ConstPtr& empty) 
{
	ROS_INFO("%s: Updating parameter server with current param vals", name.c_str());
	updateRegisterParams();
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

void SDKNode::initRegisterParams()
{
	constexpr uint32_t REG_INIT_TIMEOUT_VAL = 50000; // 50ms
	for (auto reg : configurable_regs)
	{
		reg_val_t tmp;
		const std::string reg_name = reg->getName();
		const std::string param_name = getNodeParamName(reg_name);
		// Parameters for register values are always in the node's "private" namespace, so use n_priv
		if (true == n_priv.getParam(param_name, tmp))
		{
			ROS_INFO("Setting %s register to 0x%x from param %s", reg_name.c_str(), tmp, param_name.c_str());
			reg->setVal(tmp, REG_INIT_TIMEOUT_VAL);
		}
		else
		{
			reg->getVal(&tmp, REG_INIT_TIMEOUT_VAL);
			ROS_WARN("Unable to init %s register from param %s, using existing val 0x%x instead", reg_name.c_str(), param_name.c_str(), tmp);
			// And attempt write it back so that the config file has something for next time
			n_priv.setParam(param_name, tmp);
		}
	}
}

void SDKNode::updateRegisterParams()
{
	constexpr uint32_t REG_SAVE_TIMEOUT_VAL = 50000; // 50ms
	for (auto reg : configurable_regs)
	{
		reg_val_t tmp;
		const std::string reg_name = reg->getName();
		const std::string param_name = getNodeParamName(reg_name);
		if (true == reg->getVal(&tmp, REG_SAVE_TIMEOUT_VAL))
		{
			n_priv.setParam(param_name, tmp);
			continue;
		}
		else
		{
			ROS_WARN("%s unable to update param %s from true register value", name.c_str(), param_name.c_str());
		}
	}
}

} // namespace Numurus