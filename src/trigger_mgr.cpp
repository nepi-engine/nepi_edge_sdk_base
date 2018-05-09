#include <ros/ros.h>
#include <ros/console.h>

#include "num_fpga.h"
#include "trigger_mgr.h"

namespace numurus
{

TriggerMgr::TriggerMgr() :
	sw_in{ADR_TRIG_SW_IN, true},
	hw_in_enable{ADR_TRIG_HW_IN_ENABLE, true},
	hw_in_param{ADR_TRIG_HW_IN_PARAM, true},
	hw_out_enable{ADR_TRIG_HW_OUT_ENABLE, true},
	hw_out_param{ADR_TRIG_HW_OUT_PARAM, true},
	hw_out_delay{ADR_TRIG_HW_OUT_DLY, true}
{
}

TriggerMgr::~TriggerMgr()
{
}

void TriggerMgr::executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_val)
{
	if (false == sw_in.setVal(trig_val->data))
	{
		ROS_ERROR_THROTTLE(1, "Trigger mgr. unable to execute s/w trigger 0x%x", trig_val->data);
	}
}

// H/W Trig. In
void TriggerMgr::setHwTrigInEnab(const std_msgs::UInt32::ConstPtr& enab_mask)
{
	if (false == hw_in_enable.setVal(enab_mask->data))
	{
		ROS_ERROR("Trigger mgr. unable to set h/w trigger in enable mask");
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
		ROS_ERROR("Trigger mgr. unable to set h/w trigger in config.");
	}
}

// H/W Trig. Out
void TriggerMgr::setHwTrigOutEnab(const std_msgs::UInt32::ConstPtr& enab_mask)
{
	if (false == hw_out_enable.setVal(enab_mask->data))
	{
		ROS_ERROR("Trigger mgr. unable to set h/w trigger in enable mask");
	}	
}

void TriggerMgr::configureHwTrigOut(const HwTrigOutCfg::ConstPtr& cfg)
{
	// Build the register value
	const reg_val_t cfg_val = 	((cfg->pulse_width 		& 0xFF) 		<< 8)	|
								((cfg->neg_polarity		& 0x1)			<< 0);
	if (false == hw_out_param.setVal(cfg_val))
	{
		ROS_ERROR("Trigger mgr. unable to set h/w trigger out config.");
	}
}

void TriggerMgr::setHwTrigOutDly(const std_msgs::UInt32::ConstPtr& dly_usecs)
{
	if (false == hw_out_delay.setVal(dly_usecs->data))
	{
		ROS_ERROR("Trigger mgr. unable to set h/w trigger out delay");
	}
}

} // namespace numurus

int main(int argc, char **argv)
{
	ROS_INFO("Starting the Trigger Manager Node");
	ros::init(argc, argv, "trigger_mgr");
	ros::NodeHandle n;

	// Initialize the trigger manager
	numurus::TriggerMgr trig_mgr;

	ros::Subscriber sw_trig_sub = n.subscribe("sw_trigger", 5, &numurus::TriggerMgr::executeSwTrig, &trig_mgr);
	ros::Subscriber hw_trig_in_enab_sub = n.subscribe("hw_trigger_in_enab", 5, &numurus::TriggerMgr::setHwTrigInEnab, &trig_mgr);
	ros::Subscriber hw_trig_in_cfg_sub = n.subscribe("hw_trigger_in_cfg", 1, &numurus::TriggerMgr::configureHwTrigIn, &trig_mgr);
	ros::Subscriber hw_trig_out_enab_sub = n.subscribe("hw_trigger_out_enab", 5, &numurus::TriggerMgr::setHwTrigOutEnab, &trig_mgr);
	ros::Subscriber hw_trig_out_cfg_sub = n.subscribe("hw_trigger_out_cfg", 1, &numurus::TriggerMgr::configureHwTrigOut, &trig_mgr); 
	ros::Subscriber hw_trig_out_dly_sub = n.subscribe("hw_trigger_out_dly", 1, &numurus::TriggerMgr::setHwTrigOutDly, &trig_mgr);
	if (!sw_trig_sub || !hw_trig_in_enab_sub || !hw_trig_in_cfg_sub || !hw_trig_out_enab_sub ||
		!hw_trig_out_cfg_sub || !hw_trig_out_dly_sub || !hw_trig_out_dly_sub)
	{
		ROS_ERROR("Trigger Manager unable to subscribe to required topics");
		return -1;
	}
	
	ros::spin();	
}