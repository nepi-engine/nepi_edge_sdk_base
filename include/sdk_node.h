#ifndef _SDK_NODE_H
#define _SDK_NODE_H

#include <string>

#include <ros/ros.h>

#include "register.h"

namespace numurus
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
	const std::string name;
	std::vector<Register*> configurable_regs;

	void initParams();
	void updateParams();

private:

	void initRegisterParams();
	void updateRegisterParams();
};

} // namespace numurus

#endif //_SDK_NODE_H