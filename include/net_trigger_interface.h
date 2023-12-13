/*
 * NEPI Dual-Use License
 * Project: nepi_edge_sdk_base
 *
 * This license applies to any user of NEPI Engine software
 *
 * Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
 * see https://github.com/numurus-nepi/nepi_edge_sdk_base
 *
 * This software is dual-licensed under the terms of either a NEPI software developer license
 * or a NEPI software commercial license.
 *
 * The terms of both the NEPI software developer and commercial licenses
 * can be found at: www.numurus.com/licensing-nepi-engine
 *
 * Redistributions in source code must retain this top-level comment block.
 * Plagiarizing this software to sidestep the license obligations is illegal.
 *
 * Contact Information:
 * ====================
 * - https://www.numurus.com/licensing-nepi-engine
 * - mailto:nepi@numurus.com
 *
 */
#ifndef _NET_TRIGGER_INTERFACE_H
#define _NET_TRIGGER_INTERFACE_H

#include <functional>

#include <trigger_interface.h>

namespace Numurus
{
/**
 * @brief      Class for a fully-s/w-controlled trigger interface
 * 			   
 * 			   This implementation handles triggering for a node in a completely s/w and network controlled
 * 			   way (in contrast to implementations that interact with some hardware/FPGA moodule). As such
 * 			   it may incur additional latency, but is valuable for nodes that can't access h/w resources for
 * 			   various reasons (e.g., running on a different processor, no FPGA in system, etc.)
 */	
class NetTriggerInterface : public TriggerInterface
{
public:
	/**
	 * @brief      Constructor (@see{TriggerInterface})
	 *
	 * @param      parent             The parent
	 * @param      parent_pub_nh      The parent pub nh
	 * @param      parent_priv_nh     The parent priv nh
	 * @param[in]  parent_trig_index  The parent trig index
	 */
	NetTriggerInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh, 
					   uint32_t parent_trig_index, std::function<void(SDKNode&)> parent_trig_callback);

	NetTriggerInterface() = delete; // No default constructor
	virtual ~NetTriggerInterface();

	// inherited from SDKInterface
	virtual void initSubscribers() override;

	/**
	 * @brief      Call the parent node's callback if the input trig mask contains our index 
	 * 
	 * 			   This method serves as the callback to the ROS sw_trigger topic. The node, itself
	 *
	 * @param[in]  trig_mask  The trigger mask value
	 */	
	void executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_mask);

private:
	/**
	 * Callback assigned at construction for the parent's trigger method
	 */
	std::function<void(SDKNode&)> _parent_trig_callback;

}; // class 
} // namespace Numurus

#endif