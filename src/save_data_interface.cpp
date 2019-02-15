#include "save_data_interface.h"
#include "sdk_utils.h"

#include "num_sdk_msgs/SaveDataStatus.h"

namespace Numurus
{
SaveDataInterface::SaveDataInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh):
	SDKInterface{parent, parent_pub_nh, parent_priv_nh},
	_save_continuous{"save_data_continuous", false, parent},
	_save_raw{"save_data_raw", false, parent}
{
	ROS_ERROR_STREAM("Must provide a better default file save location. Currently " << _save_data_dir);
}

SaveDataInterface::~SaveDataInterface(){}

void SaveDataInterface::retrieveParams()
{
	_save_continuous.retrieve();
	_save_raw.retrieve();
}

void SaveDataInterface::initPublishers()
{
	_save_status_pub = _parent_priv_nh->advertise<num_sdk_msgs::SaveDataStatus>("save_data_status", 5, true);
}

void SaveDataInterface::initSubscribers()
{
	// Public namespace subscriptions
	subscribers.push_back(_parent_pub_nh->subscribe("save_data", 3, &SaveDataInterface::saveDataHandler, this));
	subscribers.push_back(_parent_pub_nh->subscribe("save_data_prefix", 3, &SaveDataInterface::saveDataPrefixHandler, this));
	
	// Private namespace subscriptions
	subscribers.push_back(_parent_priv_nh->subscribe("save_data", 3, &SaveDataInterface::saveDataHandler, this));
	// TODO: Should we allow per-node prefixes?
	//subscribers.push_back(_parent_priv_nh->subscribe("save_data_prefix", 3, &SaveDataInterface::saveDataPrefixHandler, this));
}

void SaveDataInterface::saveDataHandler(const num_sdk_msgs::SaveData::ConstPtr &msg)
{
	const bool save_data_updated = (msg->save_continuous != _save_continuous) ||
								   (msg->save_raw != _save_raw);
	if (true == save_data_updated)
	{
		_save_continuous = msg->save_continuous;
		_save_raw = msg->save_raw;

		ROS_INFO("%s data save settings updated to (save_continuous=%s, save_raw=%s)", _parent_node->getName().c_str(),
				  BOOL_TO_ENABLED(_save_continuous), BOOL_TO_ENABLED(_save_raw));
	}
	// Always publish the update
	publishSaveStatus();
}

void SaveDataInterface::saveDataPrefixHandler(const std_msgs::String::ConstPtr &msg)
{
	_filename_prefix = msg->data;

	publishSaveStatus();
}

void SaveDataInterface::publishSaveStatus()
{

	num_sdk_msgs::SaveDataStatus stat_msg;
	stat_msg.current_data_dir = _save_data_dir + "/" + _filename_prefix; // TODO: Keep track of a real directory
	stat_msg.save_data.save_continuous = _save_continuous;
	stat_msg.save_data.save_raw = _save_raw;

	_save_status_pub.publish(stat_msg);
}

}