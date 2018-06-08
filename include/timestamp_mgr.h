#ifndef _TIMESTAMP_MGR_H
#define _TIMESTAMP_MGR_H

#include <ros/time.h>
#include "std_msgs/Empty.h"

#include "sdk_node.h"
#include "register.h"
#include "num_sdk_base/ConvertRawTstamp.h"

namespace Numurus
{

/**
 * @brief      Provides ROS node interface to the FPGA timestamp manager module
 * 
 * 			   This class allows the ROS system to configure and control the FPGA timestamp manager, and to resolve
 * 			   FPGA-generated timestamps to system time by tracking the synchronization between reset and sys. clock.
 */	
class TimestampMgr : public SDKNode
{
public:
	TimestampMgr();
	~TimestampMgr();

	void run() override;

	/**
	 * @brief      Gets the current FPGA timestamp in raw FPGA 1MHz counts.
	 *
	 * @param      p_raw_tstamp_out  Storage for the raw timestamp output
	 *
	 * @return     True if successful, false otherwise
	 */
	inline bool getCurrentTstampRaw(uint32_t *p_raw_tstamp_out)
	{

		return tstamp.getVal(reinterpret_cast<reg_val_t*>(p_raw_tstamp_out));
	}

	/**
	 * @brief      Gets the current FPGA timestamp as a ROS::Time object.
	 *
	 * @param      ros_time_out  Storage for the ros time output
	 *
	 * @return     True if successful, false otherwise
	 */
	bool getCurrentTstamp(ros::Time& ros_time_out);
	
	/**
	 * @brief      Gets the current FPGA tstamp in system clock seconds.
	 *
	 * @param      p_secs_out  Storage for the system clock output
	 *
	 * @return     True if successful, false otherwise
	 */
	bool getCurrentTstamp(double *p_secs_out);

	/**
	 * @brief      Resynchronize the system clock and FPGA timestamp clock (resetting FPGA timestamps to 0 in the process)
	 * 
	 * 			   This method serves as the callback for the resync_tstamp_req message.
	 *
	 * @param[in]  empty  The ROS-required empty placeholder
	 */
	void resyncToSysClock(const std_msgs::Empty::ConstPtr& empty);

	/**
	 * @brief      Convert a FPGA-provided timestamp to a ROS time
	 *
	 * 			   This method servers as the callback for the convert_raw_tstamp service
	 * @param[in]  raw_tstamp        The raw FPGA timestamp
	 * @param      converted_tstamp  The sys-clock converted timestamp as a ROS::time object
	 *
	 * @return     true if successful, false otherwise
	 */
	bool convertRawTstamp(num_sdk_base::ConvertRawTstamp::Request &raw_tstamp, num_sdk_base::ConvertRawTstamp::Response &converted_tstamp);

private:
	/**
	 * @brief      Reset the FPGA timestamp counter and capture the reset time as a sys clock value
	 *
	 * @return     true if successful, false otherwise
	 */
	bool syncFPGA();

	/**
	 * @brief      Utility to obtain the number of elapsed seconds since last syncFPGA() call
	 *
	 * @param      p_secs_out  Storage for the output
	 *
	 * @return     true if successful, false otherwise.
	 */
	bool getSecsSinceSync(double *p_secs_out);
	
	/**
	 * Most recent syncFPGA() time
	 */
	ros::Time sync_time;

	/**
	 * Prior syncFPGA() time - necessary since service callers may send pre-sync timestamps
	 */
	ros::Time sync_time_prev;

	/**
	 * Interface to the timestamp control register for FPGA timestamp reset
	 */
	Register tstamp_ctrl;

	/**
	 * Interface to the FPGA running timestamp register
	 */
	Register tstamp;
	//Register tstamp_rate;
};

} // namespace Numurus

#endif //_TIMESTAMP_MGR_H