#include <ros/ros.h>
#include <ros/console.h>

#include "num_fpga.h"
#include "timestamp_mgr.h"

#define NODE_NAME	"timestamp_mgr"

namespace Numurus
{

TimestampMgr::TimestampMgr() :
	SDKNode{NODE_NAME},
	sync_time{0},
	sync_time_prev{0},
	tstamp_ctrl{"timestamp_ctrl", ADR_TSTAMP_CTRL},
	tstamp{"timestamp", ADR_TSTAMP_CNT}
{
	ROS_DEBUG("TimestampMgr Constructor");

	if (false == tstamp_ctrl.isReady() || false == tstamp.isReady())
	{
		ROS_ERROR("Failed to set up timestamp management registers");
		return;
	}

	const bool syncRet = syncFPGA();
	if (false == syncRet)
	{
		ROS_ERROR("Failed the initial synchronization with FPGA clock");
		// Ensure that the sync objects are still 0
		sync_time.fromSec(0);
		sync_time_prev.fromSec(0);
	}
}

TimestampMgr::~TimestampMgr()
{
	ROS_DEBUG("TimestampMgr Destructor");
}

void TimestampMgr::run()
{
	init();

	// Register for the resync topic
	subscribers.push_back(n.subscribe("resync_tstamp_req", 1, &Numurus::TimestampMgr::resyncToSysClock, this));
	
	// Advertise conversion service
	servicers.push_back(n.advertiseService("convert_raw_tstamp", &Numurus::TimestampMgr::convertRawTstamp, this));

	if (false == ready())
	{
		ROS_ERROR("%s: Not properly initialized... exiting", name.c_str());
		return;		
	}

	ros::spin();
}

bool TimestampMgr::getCurrentTstamp(ros::Time& ros_time_out)
{
	double secs_since_sync;
	if (false == getSecsSinceSync(&secs_since_sync))
	{
		ros_time_out.fromSec(0);
		return false;
	}
	const ros::Duration secs_since(secs_since_sync);
	ros_time_out = sync_time + secs_since;
	return true; 
}

bool TimestampMgr::getCurrentTstamp(double *p_secs_out)
{
	double secs_since_sync;
	if (false == getSecsSinceSync(&secs_since_sync))
	{
		*p_secs_out = 0;
		return false;
	}
	*p_secs_out = secs_since_sync + sync_time.toSec();
	return true;
}

void TimestampMgr::resyncToSysClock(const std_msgs::Empty::ConstPtr& empty)
{
	if (false == syncFPGA())
	{
		ROS_ERROR("Unable to resync timestamps in response to request msg.");
	}
}

bool TimestampMgr::convertRawTstamp(num_sdk_base::ConvertRawTstamp::Request &req, num_sdk_base::ConvertRawTstamp::Response &resp)
{
	static constexpr double USECS_PER_SEC = 1000000.0;

	const double tstamp_secs_since_sync = req.raw / USECS_PER_SEC;
	const ros::Duration tstamp_since_sync(tstamp_secs_since_sync);
	reg_val_t curr_tstamp;
	if (false == tstamp.getVal(&curr_tstamp))
	{
		// Error logged upstream
		return false;
	}

	// Determine if the caller's timestamp was from before the most recent resync and adjust accordingly
	const ros::Time &offset = (req.raw > (uint32_t) curr_tstamp)?
		sync_time_prev : sync_time;
		
	resp.converted = offset + tstamp_since_sync;

	return true;
}

bool TimestampMgr::getSecsSinceSync(double *p_secs_out)
{
	static constexpr double USECS_PER_SEC = 1000000.0;
	reg_val_t curr_tstamp;
	if (false == tstamp.getVal(&curr_tstamp))
	{
		*p_secs_out = 0;
		// Error logged upstream
		return false;
	}
	
	// 
	*p_secs_out = curr_tstamp / USECS_PER_SEC;
	return true;
}

bool TimestampMgr::syncFPGA()
{
	// Set the sync_time to the average of the system time before and after the register write
	ros::Time before = ros::Time::now();
	if (false == tstamp_ctrl.setVal(1))
	{
		return false;
	}
	ros::Time after = ros::Time::now();

	sync_time_prev = sync_time;
	const double avgSecs = (before.toSec() + after.toSec()) * 0.5;
	sync_time.fromSec(avgSecs);

	ROS_INFO("Synchronized FPGA timestamps at %f secs", sync_time.toSec());
	return true;
}

} // namespace Numurus

int main(int argc, char **argv)
{
	ROS_INFO("Starting the %s Node", NODE_NAME);
	ros::init(argc, argv, NODE_NAME);
	
	// The class instance does the work
	Numurus::TimestampMgr timestamp_mgr;
	timestamp_mgr.run();

	return 0;
}
