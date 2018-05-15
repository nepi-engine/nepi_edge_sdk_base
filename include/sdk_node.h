#ifndef _SDK_NODE_H
#define _SDK_NODE_H

#include <string>

#include <ros/ros.h>

namespace numurus
{

class SDKNode
{
public:
	SDKNode(const std::string name);
	~SDKNode();

	virtual void run() = 0;
	
	inline const std::string getName() const {return name;} 

protected:
	ros::NodeHandle n;
	const std::string name;

	virtual bool initParams() {return true;}
	virtual bool saveParams() {return true;}
};

} // namespace numurus

#endif //_SDK_NODE_H