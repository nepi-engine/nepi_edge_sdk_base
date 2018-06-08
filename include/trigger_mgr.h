#ifndef _TRIGGER_MGR_H
#define _TRIGGER_MGR_H

#include <string>

#include "std_msgs/UInt32.h"

#include "num_sdk_base/HwTrigInCfg.h"
#include "num_sdk_base/HwTrigOutCfg.h"
#include "register.h"
#include "sdk_node.h"

using namespace num_sdk_base; // Messages are automatically namespaced by their project's name

namespace Numurus
{

class TriggerMgr : public SDKNode
{
public:
	TriggerMgr(const std::string name);
	~TriggerMgr();

	void run() override;
	
	// Message Callbacks
	// S/W Trig.
	void executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_mask);

	// H/W Trig. In
	void setHwTrigInEnab(const std_msgs::UInt32::ConstPtr& enab_mask);
	void configureHwTrigIn(const HwTrigInCfg::ConstPtr& cfg);
	
	// H/W Trig. Out
	void setHwTrigOutEnab(const std_msgs::UInt32::ConstPtr& enab_mask);
	void configureHwTrigOut(const HwTrigOutCfg::ConstPtr& cfg);
	void setHwTrigOutDly(const std_msgs::UInt32::ConstPtr& dly_usecs);

private:
	Register sw_in;
	Register hw_in_enable;
	Register hw_in_param;
	Register hw_out_enable;
	Register hw_out_param;
	Register hw_out_delay;

};

} // namespace Numurus

#endif //_TRIGGER_MGR_H