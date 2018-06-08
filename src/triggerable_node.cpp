#include "triggerable_node.h"

namespace Numurus
{
TriggerableNode::TriggerableNode(const std::string name, reg_addr_t trig_mask_reg_addr, reg_addr_t trig_delay_reg_addr) :
	SDKNode{name},
	trig_mask{name + "_trig_mask", trig_mask_reg_addr},
	trig_delay{name + "_trig_delay", trig_delay_reg_addr}
{
	configurable_regs.push_back(&trig_mask);
	configurable_regs.push_back(&trig_delay);
}

TriggerableNode::~TriggerableNode()
{}

void TriggerableNode::init()
{
	SDKNode::init();

	// Use the n_priv node handle to get these "generic" messages namespaced under this node
	subscribers.push_back(n_priv.subscribe("set_trig_mask", 1, &TriggerableNode::setTrigMask, this));
	subscribers.push_back(n_priv.subscribe("set_trig_delay", 1, &TriggerableNode::setTrigDelay, this));
}

void TriggerableNode::setTrigMask(const std_msgs::UInt32::ConstPtr& trig_mask_val)
{
	if (false == trig_mask.setVal(trig_mask_val->data))
	{
		ROS_ERROR_THROTTLE(1, "%s unable to set s/w trig. mask 0x%x", name.c_str(), trig_mask_val->data);
	}
}

void TriggerableNode::setTrigDelay(const std_msgs::UInt32::ConstPtr& trig_delay_val_usecs)
{
	if (false == trig_delay.setVal(trig_delay_val_usecs->data))
	{
		ROS_ERROR_THROTTLE(1, "%s unable to set s/w trigger delay %d usecs", name.c_str(), trig_delay_val_usecs->data);
	}
}

} // namespace Numurus