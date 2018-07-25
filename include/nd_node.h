#ifndef _ND_NODE_H
#define _ND_NODE_H

#include <string>

#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

#include "sdk_node.h"

#include "num_sdk_base/NDSaveData.h"
#include "num_sdk_base/NDReset.h"
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
	
	virtual void init() override;
	virtual void initParams() override;

	// Generic subscription callbacks. In many cases, the default implementation in this base class will
	// be sufficient, but subclasses can override as necessary (ensuring that they call back to the base
	// class or embed the base class logic as necessary).
	virtual void pauseEnableHandler(const std_msgs::Bool::ConstPtr &msg);
	virtual void saveDataHandler(const num_sdk_base::NDSaveData::ConstPtr &msg);
	virtual void resetHandler(const num_sdk_base::NDReset::ConstPtr &msg);
	void saveCfgRtHandler(const std_msgs::Bool::ConstPtr &msg);
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
	bool _simulated_data = false;
	
	bool _save_continuous = false;
	bool _save_raw = false;
	
	bool _save_cfg_rt = false;
	
	float _min_range = 0.0f;
	float _max_range = 1.0f;

	float _angle_offset = 0.0f;
	float _total_angle = 0.0f;

	bool _manual_resolution_enabled = true;
	float _manual_resolution = 1.0f;

	bool _gain_enabled = true;
	float _gain = 1.0f;

	bool _filter_enabled = true;
	float _filter_control = 1.0f;

	const std::string _device_type;
	const std::string _serial_num;
	const std::string _sensor_type;
	std::string _display_name;
	std::string _save_data_dir = "/var/volatile";

	ros::Publisher _status_pub;

	void publishStatus();
};

}

#endif