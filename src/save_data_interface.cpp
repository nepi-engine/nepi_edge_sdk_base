/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <sys/stat.h>
#include "boost/filesystem.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include "save_data_interface.h"
#include "sdk_utils.h"

#include "nepi_ros_interfaces/SaveDataStatus.h"
#include "nepi_ros_interfaces/SystemStorageFolderQuery.h"
#include "std_msgs/Empty.h"


namespace Numurus
{
SaveDataInterface::SaveDataInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh):
	SDKInterface{parent, parent_pub_nh, parent_priv_nh},
	_save_data_dir{""},
	_data_uid{0},
	_data_gid{0},
	_save_continuous{"save_data_continuous", false, parent},
	_save_raw{"save_data_raw", false, parent}
{
	// First, get the data directory
	if (false == ros::service::waitForService("system_storage_folder_query", 20000)) // Timeout is in ms, so 20 seconds
	{
		ROS_ERROR("Failed to obtain system storage folder");
	}
	else
	{
		ros::ServiceClient data_folder_query_client = parent_pub_nh->serviceClient<nepi_ros_interfaces::SystemStorageFolderQuery>("system_storage_folder_query");
		nepi_ros_interfaces::SystemStorageFolderQuery data_folder_query;
		data_folder_query.request.type = "data";
		if (false == data_folder_query_client.call(data_folder_query))
		{
			ROS_ERROR("Failed to obtain system data folder");
		}
		else
		{
			_save_data_dir = data_folder_query.response.folder_path;
		}
	}

	// Check that the data directory actually exists
	boost::filesystem::path p(_save_data_dir);
	if (false == boost::filesystem::exists(p))
	{
		ROS_ERROR("Reported data folder does not exist... data saving is disabled");
		_save_data_dir = ""; // Set it back to blank
	}
	else
	{
		// Ensure we know the correct user and group for folder creation later
		struct stat stat_buf;
		if (0 != stat(_save_data_dir.c_str(), &stat_buf))
		{
			ROS_ERROR("Unable to obtain ownership details of Data folder");
		}
		else
		{
			_data_uid = stat_buf.st_uid;
			_data_gid = stat_buf.st_gid;
		}
	}
}

SaveDataInterface::~SaveDataInterface(){}

void SaveDataInterface::retrieveParams()
{
	_save_continuous.retrieve();
	_save_raw.retrieve();
	//_new_save_triggered = _save_continuous;
}

void SaveDataInterface::initPublishers()
{
	if (_save_data_dir.length() != 0)
	{
		_save_status_pub = _parent_priv_nh->advertise<nepi_ros_interfaces::SaveDataStatus>("save_data_status", 5, true);
	}
}

void SaveDataInterface::initSubscribers()
{
	if (_save_data_dir.length() != 0)
	{
		// Public namespace subscriptions
		subscribers.push_back(_parent_pub_nh->subscribe("save_data", 3, &SaveDataInterface::saveDataHandler, this));
		subscribers.push_back(_parent_pub_nh->subscribe("save_data_prefix", 3, &SaveDataInterface::saveDataPrefixPubNSHandler, this));
		subscribers.push_back(_parent_pub_nh->subscribe("save_data_rate", 3, &SaveDataInterface::saveDataRateHandler, this));
		subscribers.push_back(_parent_pub_nh->subscribe("snapshot_trigger", 3, &SaveDataInterface::snapshotTriggerHandler, this));

		// Private namespace subscriptions
		subscribers.push_back(_parent_priv_nh->subscribe("save_data", 3, &SaveDataInterface::saveDataHandler, this));
		subscribers.push_back(_parent_priv_nh->subscribe("save_data_prefix", 3, &SaveDataInterface::saveDataPrefixPrivNSHandler, this));
		subscribers.push_back(_parent_priv_nh->subscribe("save_data_rate", 3, &SaveDataInterface::saveDataRateHandler, this));
		subscribers.push_back(_parent_priv_nh->subscribe("snapshot_trigger", 3, &SaveDataInterface::snapshotTriggerHandler, this));
	}
}

void SaveDataInterface::initServices()
{
	if (_save_data_dir.length() != 0)
	{
		servicers.push_back(_parent_priv_nh->advertiseService("query_data_products", &SaveDataInterface::queryDataProductsHandler, this));
	}
}

bool SaveDataInterface::saveIFReady()
{
	if (_save_data_dir.length() == 0)
	{
		return false;
	}
	return true;
}


void SaveDataInterface::registerDataProduct(const std::string product_name, double save_rate_hz, double max_save_rate_hz)
{
		if (save_rate_hz > max_save_rate_hz)
		{
			save_rate_hz = max_save_rate_hz;
		}
		float rate_float = (float) save_rate_hz;
		data_product_registry[product_name] = {save_rate_hz, 0.0, max_save_rate_hz, 0.0};
		ROS_INFO("Registered new data product %s, with %f hz", product_name.c_str(),rate_float);
		publishSaveStatus();
}

bool SaveDataInterface::dataProductRegistered(const std::string product_name)
{
	data_product_registry_entry_t entry;
	try
	{
		entry = data_product_registry.at(product_name);
	}
	catch (int e)
	{
		// Unregistered data product -- just return false
		// TODO: Log warning?
		return false;
	}
	return true;
}

bool SaveDataInterface::dataProductShouldSave(const std::string product_name, ros::Time data_time)
{
	if (_save_data_dir.length() == 0)
	{
		return false;
	}

	data_product_registry_entry_t entry;
	try
	{
		entry = data_product_registry.at(product_name);
	}
	catch (int e)
	{
		// Unregistered data product -- just return false
		// TODO: Log warning?
		return false;
	}

	const float rate_hz = entry[0];
	if (0 == rate_hz)
	{
		// Saving disabled for this data product
		return false;
	}

	const double save_time = entry[1];
	const double data_time_s = data_time.toSec();
	if (data_time_s >= save_time)
	{
		// Update for the next save time
		data_product_registry[product_name] = {rate_hz, data_time_s + (1.0/rate_hz), entry[2], entry[3]};
		return true;
	}
	// Otherwise, not time yet
	return false;
}

bool SaveDataInterface::dataProductSavingEnabled(const std::string product_name)
{
	data_product_registry_entry_t entry;
	try
	{
		entry = data_product_registry.at(product_name);
	}
	catch (int e)
	{
		// Unregistered data product -- just return false
		// TODO: Log warning?
		return false;
	}

	const float rate_hz = entry[0];
	if (0 == rate_hz)
	{
		// Saving disabled for this data product
		return false;
	}
	return true;	
}

bool SaveDataInterface::snapshotEnabled(const std::string product_name)
{
	data_product_registry_entry_t entry;
	try
	{
		entry = data_product_registry.at(product_name);
	}
	catch (int e)
	{
		// Unregistered data product -- just return false
		// TODO: Log warning?
		return false;
	}

	const float snapshot = entry[3];
	if (snapshot > 0)
	{
		// Saving disabled for this data product
		return true;
	}
	return false;	
}

void SaveDataInterface::snapshotReset(const std::string product_name)
{
	data_product_registry_entry_t entry;
	try
	{
		entry = data_product_registry.at(product_name);
	}
	catch (int e)
	{
		// Unregistered data product -- just return false
		// TODO: Log warning?
		return;
		
	}

	data_product_registry[product_name] = {0.0, entry[3]};
	
}



bool SaveDataInterface::newSaveTriggered()
{
	if (_save_data_dir.length() == 0)
	{
		return false;
	}

	bool should_save = false;
	if (_new_save_triggered == true)
	{
		_new_save_triggered = false;
		should_save = true;
	}
	return should_save;
}

std::string SaveDataInterface::getTimestampString(const ros::Time &timestamp)
{
	boost::posix_time::ptime posix_time = timestamp.toBoost();

	std::string iso_string_tstamp = boost::posix_time::to_iso_extended_string(posix_time);
	// Now improve the format of this string.
	// First, delete all ':' chars -- Windows (Samba) doesn't accept them in filenames
	size_t colon_pos = iso_string_tstamp.find(":");
	while (colon_pos != std::string::npos)
	{
		iso_string_tstamp.erase(colon_pos, 1);
		colon_pos = iso_string_tstamp.find(":");
	}

	// Now, replace a comma subsecond delimiter with a period
	/* Actually, boost documentation is wrong -- there is no comma, just a period for the subsecond delim
	const size_t comma_pos = iso_string_tstamp.find(",");
	if (comma_pos == std::string::npos) // no comma ==> no subseconds
	{
		return (iso_string_tstamp + ".000");
	}
	iso_string_tstamp.replace(comma_pos, 1, ".");
	*/

	// Now truncate the string to have at most 3 characters after the period (subseconds)
	// E.g, 2018-01-28T201408.749
	return iso_string_tstamp.substr(0, 21);
}

const std::string SaveDataInterface::getFullPathFilename(const std::string &timestamp_string, const std::string &identifier, const std::string &extension)
{
	if (_save_data_dir.length() == 0)
	{
		return "";
	}
	
	return (_save_data_dir + "/" + _filename_prefix + timestamp_string + "_" + identifier + "." + extension);
}

const std::string SaveDataInterface::getSavePrefixString()
{
	return (_filename_prefix);
}


void SaveDataInterface::saveDataHandler(const nepi_ros_interfaces::SaveData::ConstPtr &msg)
{
	const bool save_data_updated = (msg->save_continuous != _save_continuous) ||
								   (msg->save_raw != _save_raw);
	if (true == save_data_updated)
	{
		if ((msg->save_continuous == true) && (_save_continuous == false)) // Must be enabling data saving
		{
			_new_save_triggered = true;
		}

		_save_continuous = msg->save_continuous;
		_save_raw = msg->save_raw;
		ROS_INFO("%s data save settings updated to (save_continuous=%s, save_raw=%s)", _parent_node->getUnqualifiedName().c_str(),
				  BOOL_TO_ENABLED(_save_continuous), BOOL_TO_ENABLED(_save_raw));
	}
	// Always publish the update
	publishSaveStatus();
}

void SaveDataInterface::saveDataPrefixPubNSHandler(const std_msgs::String::ConstPtr &msg)
{
	_filename_prefix = msg->data;

	// Mark for calibration save if this will be in a subdirectory
	// TODO: Better would be if we detected that this was a *new* subdirectory, but that logic is more complicated and
	// saving calibration more often than necessary seems pretty benign
	if (_filename_prefix.find('/') != std::string::npos)
	{
		_new_save_triggered = true;
	}
	//TODO: Should we monitor here to ensure that a new folder gets created by system_mgr if required according to the new prefix before proceeding?

	publishSaveStatus();
}

void SaveDataInterface::saveDataPrefixPrivNSHandler(const std_msgs::String::ConstPtr &msg)
{
	_filename_prefix = msg->data;

	// Need to check if this defines a subdirectory and if so, ensure it exists
	const std::string new_path = _save_data_dir + "/" + _filename_prefix;
	boost::filesystem::path p(new_path);
	boost::filesystem::path parent_p = p.parent_path();

	if (false == boost::filesystem::exists(parent_p))
	{
		// Using boost because the equivalent std::filesystem::create_directories only exists since C++17
		boost::filesystem::create_directories(parent_p);
		// Mark that we should save calibration to the new folder
		_new_save_triggered = true;

		if (0 != chown(parent_p.string().c_str(), _data_uid, _data_gid))
		{
			ROS_ERROR("Unable to set ownership of the new %s folder (%s)", parent_p.string().c_str(), strerror(errno));
		}
	}

	publishSaveStatus();
}

void SaveDataInterface::saveDataRateHandler(const nepi_ros_interfaces::SaveDataRate::ConstPtr &msg)
{
	if (msg->save_rate_hz < 0.0)
	{
		ROS_ERROR("Can't set a negative save rate... aborting");
		return;
	}

	// Handle the special ALL indicator
	if (msg->data_product == nepi_ros_interfaces::SaveDataRate::ALL_DATA_PRODUCTS)
	{
		for (std::pair<std::string, data_product_registry_entry_t> entry : data_product_registry)
		{
			entry.second[0] = (msg->save_rate_hz <= entry.second[2])? msg->save_rate_hz : entry.second[2]; // Ensure max_save_rate is respected
			entry.second[1] = 0.0;
			float rate_float = (float) entry.second[0];
			std::string product_name = entry.first;
			ROS_INFO("Updating data product %s, to %f hz", product_name.c_str(),rate_float);
			data_product_registry[entry.first] = entry.second;
		}
		ROS_WARN("Updated ALL data product save rates");

	}
	else
	{
		try
		{
			data_product_registry_entry_t entry = data_product_registry.at(msg->data_product);
			entry[0] = (msg->save_rate_hz <= entry[2])? msg->save_rate_hz : entry[2]; // Ensure max_save_rate is respected
			entry[1] = 0.0;
			float rate_float = (float) entry[0];
			std::string product_name = msg->data_product;
			data_product_registry[msg->data_product] = entry;
			ROS_INFO("Updating data product %s, to %f hz", product_name.c_str(),rate_float);
		}
		catch (...)
		{
			registerDataProduct(msg->data_product,msg->save_rate_hz,100);
			ROS_WARN("Added data product to registry %s", msg->data_product.c_str());
		}
		ROS_WARN("Updated save rate for data product %s", msg->data_product.c_str());
	}
	publishSaveStatus();
}


void SaveDataInterface::snapshotTriggerHandler(const std_msgs::Empty::ConstPtr &msg)
{

	for (std::pair<std::string, data_product_registry_entry_t> entry : data_product_registry)
	{
		if (entry.second[0] > 0)
		{
		entry.second[3] = 1.0;
		}

	}
	
}

bool SaveDataInterface::queryDataProductsHandler(nepi_ros_interfaces::DataProductQuery::Request &req, nepi_ros_interfaces::DataProductQuery::Response &res)
{
	for (std::pair<std::string, data_product_registry_entry_t> entry : data_product_registry)
	{
		nepi_ros_interfaces::SaveDataRate d;
		d.data_product = entry.first;
		d.save_rate_hz = entry.second[0];
		res.data_products.push_back(d);
	}
	return true;
}

void SaveDataInterface::publishSaveStatus()
{

	std::string prefixDirName = "";
	std::string prefixFileName = "";
	std::string delimiter = "/";
	if (_filename_prefix.find('a') == _filename_prefix.npos)
	{
		prefixFileName = _filename_prefix;
	
	}
	else 
	{
		prefixDirName = _filename_prefix.substr(0, _filename_prefix.find(delimiter));
		prefixFileName = _filename_prefix.substr(1, _filename_prefix.find(delimiter));
	}


	 std::string key_str = "";
	 double val = 0.0;
	 std::string rate_entry = "";
	 std::string saveRates = "[";
	for (std::pair<std::string, data_product_registry_entry_t> entry : data_product_registry)
		{   
			key_str = entry.first;
			val = entry.second[0];
			float val_float = (float) val;
			std::string val_str = std::to_string(val_float);
			rate_entry = "[" + key_str + "," + val_str + "],";
			saveRates = saveRates + rate_entry;
		}

	saveRates = saveRates + "]";

	nepi_ros_interfaces::SaveDataStatus stat_msg;
	stat_msg.current_data_dir = _save_data_dir; 
	stat_msg.current_folder_prefix = prefixDirName;
	stat_msg.current_filename_prefix = prefixFileName;
	stat_msg.save_data_rates = saveRates;
	stat_msg.save_data.save_continuous = _save_continuous;
	stat_msg.save_data.save_raw = _save_raw;

	_save_status_pub.publish(stat_msg);

}

}
