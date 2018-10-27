#include "num_sdk_msgs/TriggerIndexSettings.h"

#include "trigger_interface.h"

namespace Numurus
{
TriggerInterface::TriggerInterface(SDKNode *parent, uint32_t parent_trig_index):
	trig_enabled{true},
	trig_index{parent_trig_index},
	//trig_mask{"trig_mask", 0, parent},
	trig_delay{"trig_delay", 0, parent},
	nh{parent->getPublicNamespace()}
{
	
	// Advertise and publish (with latch) the trigger index so trig mgr. can populate trig defs appropriately
	trig_index_pub = nh.advertise<num_sdk_msgs::TriggerIndexSettings>("trigger_index_settings", 5, true); // true to latch it; ensure it is sent on each new subscriber connection

	num_sdk_msgs::TriggerIndexSettings index_settings;
	index_settings.trigger_name = parent->getDisplayName(); // Use display name for configurability
	index_settings.index = trig_index;
	
	trig_index_pub.publish(index_settings);
}

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
