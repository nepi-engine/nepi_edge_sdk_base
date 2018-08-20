#ifndef _ND_NODE_H
#define _ND_NODE_H

#include <string>

#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

#include "sdk_node.h"

#include "num_sdk_base/NDSaveData.h"
#include "num_sdk_base/NDRange.h"
#include "num_sdk_base/NDAngle.h"
#include "num_sdk_base/NDAutoManualSelection.h"
#include "num_sdk_base/NDStatus.h"

namespace Numurus
{
// Forward declarations
class TriggerInterface;

/**
 * @brief      The NDNode class represents a Numurus sensor capable of producing multidimensional imagery
 * 
 * 			   All NDNodes adhere to a common ROS interface in the form of guaranteed published and 
 * 			   subscribed topics. This class provides a base from which sensor-specific implementation can
 * 			   be derived.
 * 			   
 * @note:	   This class inherits from abstract SDKNode, and does not implement all of the pure virtual 
 * 			   methods so, it is, itself, abstract.
 */
class NDNode : public SDKNode
{
public:
	/**
	 * @brief      Constructor
	 *
	 * @param[in]  sensor_name  The node name (as factory-set, not readily configurable)
	 * @param[in]  dev_type     The device time (as factory-set, not readily configurable)
	 * @param[in]  serial_num   The serial number (as factory-set, not readily configurable)
	 * @param[in]  sensor_type  The sensor type (as factory-set, not readily configurable)
	 */
	NDNode( const std::string sensor_name, const std::string dev_type, 
			const std::string serial_num, const std::string sensor_type);

protected:
	TriggerInterface *_trig_if = nullptr;
	
	// Inherited from SDKNode
	virtual void initPublishers() override;
	virtual void retrieveParams() override;
	virtual void initSubscribers() override;
	
	// Generic subscription callbacks. In many cases, the default implementation in this base class will
	// be sufficient, but subclasses can override as necessary (ensuring that they call back to the base
	// class or embed the base class logic as necessary).
	virtual void pauseEnableHandler(const std_msgs::Bool::ConstPtr &msg);
	virtual void saveDataHandler(const num_sdk_base::NDSaveData::ConstPtr &msg);
	void simulateDataHandler(const std_msgs::Bool::ConstPtr &msg);

	// Node-specific subscription callbacks. Concrete instances should define what actions these take,
	// though we provide a very basic private member setter implementation in this baseclass
	virtual void setRangeHandler(const num_sdk_base::NDRange::ConstPtr &msg);
	virtual void setAngleHandler(const num_sdk_base::NDAngle::ConstPtr &msg);
	virtual void setResolutionHandler(const num_sdk_base::NDAutoManualSelection::ConstPtr &msg);
	virtual void setGainHandler(const num_sdk_base::NDAutoManualSelection::ConstPtr &msg);
	virtual void setFilterHandler(const num_sdk_base::NDAutoManualSelection::ConstPtr &msg);

private:
	bool _paused = false;
	SDKNode::NodeParam<bool> _simulated_data;
	
	SDKNode::NodeParam<bool> _save_continuous;
	SDKNode::NodeParam<bool> _save_raw;
	
	SDKNode::NodeParam<float> _min_range;
	SDKNode::NodeParam<float> _max_range;

	SDKNode::NodeParam<float> _angle_offset;
	SDKNode::NodeParam<float> _total_angle;

	SDKNode::NodeParam<bool> _manual_resolution_enabled;
	SDKNode::NodeParam<float> _manual_resolution;

	SDKNode::NodeParam<bool> _gain_enabled;
	SDKNode::NodeParam<float> _gain;

	SDKNode::NodeParam<bool> _filter_enabled;
	SDKNode::NodeParam<float> _filter_control;

	const std::string _device_type;
	const std::string _serial_num;
	const std::string _sensor_type;
	std::string _save_data_dir = "/var/volatile";

	ros::Publisher _status_pub;

	void publishStatus();
};

}

#endif