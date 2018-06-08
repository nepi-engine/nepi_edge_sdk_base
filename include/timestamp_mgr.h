#ifndef _TIMESTAMP_MGR_H
#define _TIMESTAMP_MGR_H

#include <ros/time.h>
#include "std_msgs/Empty.h"

#include "sdk_node.h"
#include "register.h"
#include "num_sdk_base/ConvertRawTstamp.h"

namespace Numurus
{

class TimestampMgr : public SDKNode
{
public:
	TimestampMgr(const std::string name);
	~TimestampMgr();

	void run() override;

	// System timestamp access
	inline bool getCurrentTstampRaw(uint32_t *p_raw_tstamp_out)
	{

		return tstamp.getVal(reinterpret_cast<reg_val_t*>(p_raw_tstamp_out));
	}
	bool getCurrentTstamp(ros::Time& ros_time_out);
	bool getCurrentTstamp(double *p_secs_out);

	// Msg Callbacks
	void resyncToSysClock(const std_msgs::Empty::ConstPtr& empty);

	// Service Callbacks
	bool convertRawTstamp(num_sdk_base::ConvertRawTstamp::Request &raw_tstamp, num_sdk_base::ConvertRawTstamp::Response &converted_tstamp);

	// Utilities
	bool getSecsSinceSync(double *p_secs_out);


private:
	bool syncFPGA();
	
	ros::Time sync_time;
	ros::Time sync_time_prev;

	Register tstamp_ctrl;
	Register tstamp;
	//Register tstamp_rate;
};

} // namespace Numurus

#endif //_TIMESTAMP_MGR_H