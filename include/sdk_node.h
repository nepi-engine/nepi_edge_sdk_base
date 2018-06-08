#ifndef _SDK_NODE_H
#define _SDK_NODE_H

#include <string>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "num_common.h"
#include "register.h"

namespace Numurus
{

class SDKNode
{
public:
	SDKNode(const std::string name);
	~SDKNode();

	virtual void run() = 0;
	
	inline const std::string& getName() const {return name;} 

protected:
	ros::NodeHandle n;
	ros::NodeHandle n_priv; // Private node handle for namespace resolution
	const std::string name;
	std::vector<Register*> configurable_regs;
	std::vector<ros::ServiceServer> servicers;
	std::vector<ros::Subscriber> subscribers;

	inline const std::string getNodeParamName(const std::string &param) const
	{
		static const std::string ns_separator("/");
		return BASE_ROS_NAMESPACE + ns_separator + name + ns_separator + param;
	}

	virtual void init();

	void reinitParams(const std_msgs::String::ConstPtr& id);
	void updateParams(const std_msgs::String::ConstPtr& id);

	virtual bool ready();

private:
	bool initialized;

	void initParams();
	void initRegisterParams();
	void updateRegisterParams();
};

} // namespace Numurus

#endif //_SDK_NODE_H