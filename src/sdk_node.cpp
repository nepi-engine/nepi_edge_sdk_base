
#include "sdk_node.h"
#include "num_common.h"

namespace numurus
{

SDKNode::SDKNode(const std::string my_name) :
	name{my_name}
{}

SDKNode::~SDKNode()
{}

void SDKNode::initParams()
{
	initRegisterParams();
	// TODO: Non-register params
}

void SDKNode::updateParams() 
{
	updateRegisterParams();
	// TODO: Non-register params
}

void SDKNode::initRegisterParams()
{
	constexpr uint32_t REG_INIT_TIMEOUT_VAL = 50000; // 50ms
	for (auto reg : configurable_regs)
	{
		reg_val_t tmp;
		const std::string reg_name = reg->getName();
		if (true == n.getParam(reg_name, tmp))
		{
			ROS_INFO("%s setting %s register to 0x%x from config. file", name.c_str(), reg->getName().c_str(), tmp);
			reg->setVal(tmp, REG_INIT_TIMEOUT_VAL);
		}
		else
		{
			reg->getVal(&tmp, REG_INIT_TIMEOUT_VAL);
			ROS_WARN("%s unable to init %s register from stored config., using existing val 0x%x", name.c_str(), reg->getName().c_str(), tmp);
			// And attempt write it back so that the config file has something for next time
			n.setParam(reg_name, tmp);
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
		if (true == reg->getVal(&tmp, REG_SAVE_TIMEOUT_VAL))
		{
			n.setParam(reg_name, tmp);
			continue;
		}
		else
		{
			ROS_WARN("%s unable to update param %s from true register value", name.c_str(), reg->getName().c_str());
		}
	}
}

} // namespace numurus