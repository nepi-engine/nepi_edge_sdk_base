#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include "num_common.h"
#include "num_fpga.h"
#include "trigger_mgr.h"

#define NODE_NAME	"trigger_mgr"

namespace Numurus
{

TriggerMgr::TriggerMgr(const std::string my_name) :
	SDKNode{my_name},
	sw_in{"sw_trig_in", ADR_TRIG_SW_IN},
	hw_in_enable{"hw_trig_in_enab", ADR_TRIG_HW_IN_ENABLE},
	hw_in_param{"hw_trig_in_param", ADR_TRIG_HW_IN_PARAM},
	hw_out_enable{"hw_trig_out_enab", ADR_TRIG_HW_OUT_ENABLE},
	hw_out_param{"hw_trig_out_param", ADR_TRIG_HW_OUT_PARAM},
	hw_out_delay{"hw_trig_out_delay", ADR_TRIG_HW_OUT_DLY}
{
	configurable_regs.push_back(&hw_in_enable);
	configurable_regs.push_back(&hw_in_param);
	configurable_regs.push_back(&hw_out_enable);
	configurable_regs.push_back(&hw_out_param);
	configurable_regs.push_back(&hw_out_delay);
}

TriggerMgr::~TriggerMgr()
{}

void TriggerMgr::run()
{
	init();

	//TODO: Must advertise services like updateParams(definitely) and reinitParams(maybe)

	// Setup messages, services, and actions
	subscribers.push_back(n.subscribe("sw_trigger", 5, &numurus::TriggerMgr::executeSwTrig, this));
	subscribers.push_back(n.subscribe("hw_trigger_in_enab", 5, &numurus::TriggerMgr::setHwTrigInEnab, this));
	subscribers.push_back(n.subscribe("hw_trigger_in_cfg", 1, &numurus::TriggerMgr::configureHwTrigIn, this));
	subscribers.push_back(n.subscribe("hw_trigger_out_enab", 5, &numurus::TriggerMgr::setHwTrigOutEnab, this));
	subscribers.push_back(n.subscribe("hw_trigger_out_cfg", 1, &numurus::TriggerMgr::configureHwTrigOut, this)); 
	subscribers.push_back(n.subscribe("hw_trigger_out_dly", 1, &numurus::TriggerMgr::setHwTrigOutDly, this));
	if (false == ready())
	{
		ROS_ERROR("%s: not properly initialized... exiting", name.c_str());
		return;		
	}
	
	ros::spin();
}

void TriggerMgr::executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_val)
{
	if (false == sw_in.setVal(trig_val->data))
	{
		ROS_ERROR_THROTTLE(1, "%s unable to execute s/w trigger 0x%x", name.c_str(), trig_val->data);
	}
}

// H/W Trig. In
void TriggerMgr::setHwTrigInEnab(const std_msgs::UInt32::ConstPtr& enab_mask)
{
	if (false == hw_in_enable.setVal(enab_mask->data))
	{
		ROS_ERROR("%s unable to set h/w trigger in enable mask", name.c_str());
	}
}

void TriggerMgr::configureHwTrigIn(const HwTrigInCfg::ConstPtr& cfg)
{
	// Build the register value
	const reg_val_t cfg_val = 	((cfg->samp_rate 		& 0xFFFF) 		<< 16)	|
								((cfg->min_trig_length 	& 0xFF)		 	<< 8)	|
								((cfg->neg_polarity		& 0x1)			<< 0);
	if (false == hw_in_param.setVal(cfg_val))
	{
		ROS_ERROR("%s unable to set h/w trigger in config.", name.c_str());
	}
}

// H/W Trig. Out
void TriggerMgr::setHwTrigOutEnab(const std_msgs::UInt32::ConstPtr& enab_mask)
{
	if (false == hw_out_enable.setVal(enab_mask->data))
	{
		ROS_ERROR("%s unable to set h/w trigger in enable mask", name.c_str());
	}	
}

void TriggerMgr::configureHwTrigOut(const HwTrigOutCfg::ConstPtr& cfg)
{
	// Build the register value
	const reg_val_t cfg_val = 	((cfg->pulse_width 		& 0xFF) 		<< 8)	|
								((cfg->neg_polarity		& 0x1)			<< 0);
	if (false == hw_out_param.setVal(cfg_val))
	{
		ROS_ERROR("%s unable to set h/w trigger out config.", name.c_str());
	}
}

void TriggerMgr::setHwTrigOutDly(const std_msgs::UInt32::ConstPtr& dly_usecs)
{
	if (false == hw_out_delay.setVal(dly_usecs->data))
	{
		ROS_ERROR("%s unable to set h/w trigger out delay", name.c_str());
	}
}

} // namespace Numurus

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ROS_INFO("Starting the %s node", NODE_NAME);
	
	numurus::TriggerMgr trig_mgr(NODE_NAME);
	trig_mgr.run();	

	return 0;
}