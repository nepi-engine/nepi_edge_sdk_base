#ifndef _TRIGGERABLE_NODE_H
#define _TRIGGERABLE_NODE_H

#include "std_msgs/UInt32.h"

#include "sdk_node.h"

namespace numurus 
{
class TriggerableNode : public SDKNode
{
public:
	TriggerableNode(const std::string name, reg_addr_t trig_mask_reg_addr, reg_addr_t trig_delay_reg_addr);
	~TriggerableNode();

protected:
	virtual void init() override;

private:
	void setTrigMask(const std_msgs::UInt32::ConstPtr& trig_mask_val);
	void setTrigDelay(const std_msgs::UInt32::ConstPtr& trig_delay_val_usecs);

	Register trig_mask;
	Register trig_delay;
};

} // namespace numurus
#endif