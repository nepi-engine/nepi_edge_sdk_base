#include "trigger_interface.h"

namespace Numurus
{
TriggerInterface::TriggerInterface(SDKNode *parent, uint32_t parent_trig_index):
	trig_enabled{true},
	trig_index{parent_trig_index},
	//trig_mask{"trig_mask", 0, parent},
	trig_delay{"trig_delay", 0, parent}
{}

TriggerInterface::~TriggerInterface(){}

void TriggerInterface::retrieveParams()
{
	//trig_mask.retrieve();
	trig_delay.retrieve();
}

void TriggerInterface::setTrigDelay(const std_msgs::UInt32::ConstPtr& trig_delay_val_usecs)
{
	trig_delay = trig_delay_val_usecs->data;
}
	
} // namespace Numurus
