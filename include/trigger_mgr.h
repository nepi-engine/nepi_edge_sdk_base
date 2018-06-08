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

/**
 * @brief      ROS node interface to the FPGA trigger manager module
 * 
 * 			   This class provides configuration and control of the FPGA trigger manager module, which
 * 			   establishes hardware input/output trigger configuration, controls input s/w trigger masks, etc.
 */	
class TriggerMgr : public SDKNode
{
public:
	TriggerMgr();
	~TriggerMgr();

	/**
	 * @brief      Run the trigger manager interface node
	 * 
	 * 			   This method implement the pure virtual run() method from the abstract SDKNode base class. It
	 * 			   properly initializes the node, connect to relevant ROS internals (like subscribed topics), and
	 * 			   calls ros::spin() to kick off the event-driven system.
	 */
	void run() override;
	
	/**
	 * @brief      Send the s/w input trigger value to the FPGA
	 * 
	 * 			   This method serves as the callback to the ROS sw_trigger topic
	 *
	 * @param[in]  trig_mask  The trigger mask value
	 */
	void executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_mask);

	/**
	 * @brief      Enable hardware trigger input and set the output mask
	 * 
	 * 			   This method serves as a callback to the ROS hw_trig_in_enab topic
	 *			   On receipt of a hardware trigger, the FPGA will generate a signal for all s/w triggers
	 *			   that are enabled through this mask value
	 *
	 * @param[in]  enab_mask  The s/w mask message, which specifies the value to generate on receipt of hardware trigger input
	 */
	void setHwTrigInEnab(const std_msgs::UInt32::ConstPtr& enab_mask);
	
	/**
	 * @brief      Configure the hardware trigger input signal logic
	 *
	 *			   This method serves as a callback to the ROS hw_trigger_in_cfg topic
	 *			   The hardware trigger signal has polarity, debounce characteristics, etc. according to this value
	 *			   
	 * @param[in]  cfg   The configuration message
	 */
	void configureHwTrigIn(const HwTrigInCfg::ConstPtr& cfg);
	
	/**
	 * @brief      Enable the hardware trigger output and set the mask of s/w inputs that can generate a h/w trigger out
	 * 
	 * 			   This method serves as a callback for the hw_trigger_out_enab topic.
	 * 			   It establishes the set of s/w input triggers that result in a h/w output trigger      
	 *
	 * @param[in]  enab_mask  The enable mask message
	 */
	void setHwTrigOutEnab(const std_msgs::UInt32::ConstPtr& enab_mask);

	/**
	 * @brief      Configure the hardware trigger output signal
	 * 
	 *			   This method serves as a callback for the hw_trigger_out_cfg topic.
	 *			   It sets the various configurable parameters 
	 *
	 * @param[in]  cfg   The configuration message
	 */
	void configureHwTrigOut(const HwTrigOutCfg::ConstPtr& cfg);

	/**
	 * @brief      Set the h/w triggger output delay
	 * 
	 *  		   This method serves as a callback for the hw_trigger_out_delay topic
	 *  		   It sets the delay (in usecs) between receipt of a mask-enabled s/w input trigger and the corresponding output signal.
	 *
	 * @param[in]  dly_usecs  The delay message
	 */	
	void setHwTrigOutDly(const std_msgs::UInt32::ConstPtr& dly_usecs);

private:
	/**
	 * Interface for the s/w input trigger register
	 */
	Register sw_in;

	/**
	 * Interface for the h/w input trigger enable register
	 */
	Register hw_in_enable;

	/**
	 * Interface for the h/w input trigger signal configuration register
	 */
	Register hw_in_param;

	/**
	 * Interface for the h/w output trigger enable register
	 */
	Register hw_out_enable;

	/**
	 * Interface for the h/w output trigger signal configuration register
	 */
	Register hw_out_param;

	/**
	 * Interface for the hw/ output trigger delay register
	 */
	Register hw_out_delay;
};

} // namespace Numurus

#endif //_TRIGGER_MGR_H