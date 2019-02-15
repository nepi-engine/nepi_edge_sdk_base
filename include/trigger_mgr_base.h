#ifndef _TRIGGER_MGR_BASE_H
#define _TRIGGER_MGR_BASE_H

#include <string>
#include <unordered_map>
#include <mutex>

#include "ros/console.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Bool.h"

#include "num_sdk_msgs/HwTrigInCfg.h"
#include "num_sdk_msgs/HwTrigOutCfg.h"
#include "num_sdk_msgs/PeriodicSwTrig.h"
#include "num_sdk_msgs/TriggerIndexSettings.h"
#include "num_sdk_msgs/TriggerStatusQuery.h"
#include "num_sdk_msgs/TriggerDefs.h"
#include "sdk_node.h"

namespace Numurus
{

/**
 * @brief      ROS node interface to the FPGA-agnostic trigger manager base module
 * 
 * 			   This class provides a base for the configuration and control of the non-FPGA trigger manager functionality.
 * 			   It was separated from the FPGA-dependent trigger manager functionality to support running a subset of the 
 * 			   full trigger interface on an FPGA-free processor, specifically the Jetson TX2 in the 3DSC (for an initial
 * 			   ship where Zynq network interface was broken hence unsuitable to serve in the ROS subsystem).
 */	
class TriggerMgrBase : public SDKNode
{
public:
	TriggerMgrBase();
	~TriggerMgrBase();

protected:
	// Overrides from SDKNode
	virtual void initSubscribers() override;
	virtual void initPublishers() override;
	virtual void initServices() override;

	virtual void executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_mask);
	// Pure virtual h/w capability functions
	virtual void setHwTrigInEnab(const std_msgs::UInt32::ConstPtr& enab_mask) = 0;
	virtual void configureHwTrigIn(const num_sdk_msgs::HwTrigInCfg::ConstPtr& cfg) = 0;
	virtual void setHwTrigOutEnab(const std_msgs::Bool::ConstPtr& enable) = 0;
	virtual void configureHwTrigOut(const num_sdk_msgs::HwTrigOutCfg::ConstPtr& cfg) = 0;
	virtual void setHwTrigOutDly(const std_msgs::UInt32::ConstPtr& dly_usecs) = 0;

private:
	/**
	 * Map of current periodic triggers <mask, rate>
	 */
	std::unordered_map<uint32_t, float> periodic_trig_map;

	/**
	 * Map of most recent trigger execution times for each trig mask
	 */
	std::unordered_map<uint32_t, ros::Time> last_trig_time_map;
	
	/**
	 * Mutex to protect periodic_trig_map, which is accessed by main and all periodic worker threads
	 */
	std::mutex trig_map_mutex;

	/**
	 * Handle to the sw_trig publisher called by periodic tasks
	 */
	ros::Publisher _sw_trig_pub;

	/**
	 * Map of the active trigger indices (first=index, second=trigger interface name)
	 */
	std::map<uint32_t, std::string> trig_indices;

	/**
	 * @brief      Sets a periodic software trig. configuration
	 * 
	 * 			   This method serves as a callback for the set_periodic_sw_trig topic
	 * 			   It starts or stops a periodic trigger.
	 *
	 * @param[in]  trig_cfg  The trig configuration message
	 */
	void setPeriodicSwTrig(const num_sdk_msgs::PeriodicSwTrig::ConstPtr& trig_cfg);

	/**
	 * @brief      Worker method for periodic trigger threads
	 *
	 * @param[in]  trig_mask  The trig mask value for this thread
	 */
	void runPeriodicTrig(uint32_t trig_mask);

	/**
	 * @brief      Adds an entry to the trigger index set
	 * 
	 *			   This method serves as a callback for the trigger_index_settings message received from triggerable nodes 
	 *
	 * @param[in]  trig_idx_settings  The trig index settings message
	 */
	void updateTriggerIndexSet(const num_sdk_msgs::TriggerIndexSettings::ConstPtr& trig_idx_settings);

	/**
	 * @brief      Provide the trigger status
	 * 
	 * 			   This method serves as a callback for the trigger_status_query service.
	 *	
	 * @param      req   The request, includes the specific trigger value for the request
	 * @param      resp  The response
	 *
	 * @return     true if successful, false otherwise
	 */
	bool provideTriggerStatus(num_sdk_msgs::TriggerStatusQuery::Request &req, num_sdk_msgs::TriggerStatusQuery::Response &resp);

	/**
	 * @brief      Provide the trigger defs
	 * 
	 *             This method serves as a callback for the trigger_defs service
	 *
	 * @param      req   The request (empty)
	 * @param      resp  The response, includes a fixed array of trigger indices
	 *
	 * @return     true
	 */
	bool provideTriggerDefs(num_sdk_msgs::TriggerDefs::Request &req, num_sdk_msgs::TriggerDefs::Response &resp);

};

} // namespace Numurus

#endif //_TRIGGER_MGR_H