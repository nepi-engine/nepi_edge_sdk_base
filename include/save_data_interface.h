#ifndef _SAVE_DATA_INTERFACE_H
#define _SAVE_DATA_INTERFACE_H

#include <string>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <num_sdk_msgs/SaveData.h>

#include <sdk_node.h>

namespace Numurus
{

class SaveDataInterface
{
public:
	SaveDataInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh);
	virtual ~SaveDataInterface();

	void retrieveParams();
	void initPublishers();
	void initSubscribers();

protected:
	// Param-server and file-backed Parameters
	SDKNode::NodeParam<bool> _save_continuous;
	SDKNode::NodeParam<bool> _save_raw;

	/**
	 * User-configurable identifying prefix
	 */
	std::string _filename_prefix;
	/**
	 * Non-user-configurable base path
	 */
	std::string _save_data_dir = "/var/volatile";

	SDKNode *_parent_node;

	ros::NodeHandle *_parent_pub_nh;
	ros::NodeHandle *_parent_priv_nh;

	/**
	 * Vector of return handles from NodeHandle::subscribe.
	 * These handles must have a lifetime as long as the NodeHandle, so are best appropriated to a member variable container
	 */
	std::vector<ros::Subscriber> subscribers;

	ros::Publisher _save_status_pub;

	void saveDataHandler(const num_sdk_msgs::SaveData::ConstPtr &msg);

	void saveDataPrefixHandler(const std_msgs::String::ConstPtr &msg);

	void publishSaveStatus();

}; // class SaveDataInterface

} // namespace Numurus

#endif // _SAVE_DATA_INTERFACE_H