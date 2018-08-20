#include "trigger_interface.h"

namespace Numurus
{
TriggerInterface::TriggerInterface(SDKNode *parent):
	trig_enabled{true},
	trig_mask{"trig_mask", 0, parent},
	trig_delay{"trig_delay", 0, parent}
{}

TriggerInterface::~TriggerInterface(){}

void TriggerInterface::retrieveParams()
{
	trig_mask.retrieve();
	trig_delay.retrieve();
}

void TriggerInterface::setTrigMask(const std_msgs::UInt32::ConstPtr& trig_mask_val)
{
	trig_mask = trig_mask_val->data;
}

void TriggerInterface::setTrigDelay(const std_msgs::UInt32::ConstPtr& trig_delay_val_usecs)
{
	trig_delay = trig_delay_val_usecs->data;
}
	
} // namespace Numurus
