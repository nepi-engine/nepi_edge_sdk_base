#ifndef _TRIGGER_INTERFACE_H
#define _TRIGGER_INTERFACE_H

#include "std_msgs/UInt32.h"
#include "sdk_node.h"

namespace Numurus
{
/**
 * @brief      Abstract base (interface) class for nodes that can be triggered
 * 
 * 			   This class provides the basic ROS interface for nodes that support
 * 			   hardware triggering. 
 */	
class TriggerInterface
{
public:
	TriggerInterface(SDKNode *parent, uint32_t parent_trig_index);

	virtual ~TriggerInterface(); 

	/**
	 * @brief      Retrieve all ROS params from the param server
	 * 
	 * @note       See SDKNode::retrieveParams().
	 */
	virtual void retrieveParams();

	/**
	 * @brief      Enables or disables the triggering.
	 * 
	 * @param[in]	enabled		True to enable triggering, false otherwise
	 */
	virtual inline void setTrigEnabled(bool enabled){trig_enabled = enabled;}
protected:
	/**
	 * True if trigger is enabled for the parent node, false otherwise
	 */
	bool trig_enabled;
	
	/**
	 * Fixed trigger index. Specified in constructor.
	 */
	const uint32_t trig_index;
	
	SDKNode::NodeParam<int> trig_delay;

	// Create these here to keep them in scope
	ros::NodeHandle nh;
	ros::Publisher trig_index_pub;

	/**
	 * @brief      Sets the input trigger delay value (in usecs).
	 * 
	 * 			   This method serves as the callback for the (private namespace) set_trig_delay topic
	 *
	 * @param[in]  trig_delay_val_usecs  The trigger delay value to set
	 */	
	virtual void setTrigDelay(const std_msgs::UInt32::ConstPtr& trig_delay_val_usecs);

};
} // namespace Numurus
#endif //_TRIGGER_INTERFACE_H