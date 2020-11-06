#include "boost/filesystem.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

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
	subscribers.push_back(_parent_pub_nh->subscribe("save_data_rate", 3, &SaveDataInterface::saveDataRateHandler, this));

	// Private namespace subscriptions
	subscribers.push_back(_parent_priv_nh->subscribe("save_data", 3, &SaveDataInterface::saveDataHandler, this));
	subscribers.push_back(_parent_priv_nh->subscribe("save_data_prefix", 3, &SaveDataInterface::saveDataPrefixHandler, this));
	subscribers.push_back(_parent_priv_nh->subscribe("save_data_rate", 3, &SaveDataInterface::saveDataRateHandler, this));
}

void SaveDataInterface::initServices()
{
	servicers.push_back(_parent_priv_nh->advertiseService("query_data_products", &SaveDataInterface::queryDataProductsHandler, this));
}

void SaveDataInterface::registerDataProduct(const std::string product_name, float save_rate_hz)
{
	data_product_registry[product_name] = {save_rate_hz, 0.0};
}

bool SaveDataInterface::dataProductShouldSave(const std::string product_name, ros::Time data_time)
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

	const float rate_hz = entry.first;
	if (0 == rate_hz)
	{
		// Saving disabled for this data product
		return false;
	}

	const double save_time = entry.second;
	const double data_time_s = data_time.toSec();
	if (data_time_s >= save_time)
	{
		// Update for the next save time
		data_product_registry[product_name] = {rate_hz, data_time_s + (1.0/rate_hz)};
		return true;
	}
	// Otherwise, not time yet
	return false;
}

std::string SaveDataInterface::getTimestampString()
{
	boost::posix_time::ptime posix_time = boost::posix_time::microsec_clock::local_time();
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

void SaveDataInterface::saveDataHandler(const num_sdk_msgs::SaveData::ConstPtr &msg)
{
	const bool save_data_updated = (msg->save_continuous != _save_continuous) ||
								   (msg->save_raw != _save_raw);
	if (true == save_data_updated)
	{
		_save_continuous = msg->save_continuous;
		_save_raw = msg->save_raw;

		ROS_INFO("%s data save settings updated to (save_continuous=%s, save_raw=%s)", _parent_node->getUnqualifiedName().c_str(),
				  BOOL_TO_ENABLED(_save_continuous), BOOL_TO_ENABLED(_save_raw));
	}
	// Always publish the update
	publishSaveStatus();
}

void SaveDataInterface::saveDataPrefixHandler(const std_msgs::String::ConstPtr &msg)
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
	}

	publishSaveStatus();
}

void SaveDataInterface::saveDataRateHandler(const num_sdk_msgs::SaveDataRate::ConstPtr &msg)
{
	// Handle the special ALL indicator
	if (msg->data_product == num_sdk_msgs::SaveDataRate::ALL_DATA_PRODUCTS)
	{
		for (std::pair<std::string, data_product_registry_entry_t> entry : data_product_registry)
		{
			entry.second.first = msg->save_rate_hz;
			entry.second.second = 0.0;
			ROS_WARN("Updating save rate for data product %s", entry.first.c_str());
			data_product_registry[entry.first] = entry.second;
		}
		ROS_WARN("Updated ALL data product save rates");
		return;
	}

	try
	{
		data_product_registry_entry_t entry = data_product_registry.at(msg->data_product);
		entry.first = msg->save_rate_hz;
		entry.second = 0.0;
		data_product_registry[msg->data_product] = entry;
	}
	catch (...)
	{
		// No warning message since this could simply have been received when intended for other nodes because
		// of namespace hierarchy
		//ROS_WARN("Cannot update save rate for unregistered data product %s", msg->data_product.c_str());
		return;
	}
	ROS_WARN("Updated save rate for data product %s", msg->data_product.c_str());
	return;
}

bool SaveDataInterface::queryDataProductsHandler(num_sdk_msgs::DataProductQuery::Request &req, num_sdk_msgs::DataProductQuery::Response &res)
{
	for (std::pair<std::string, data_product_registry_entry_t> entry : data_product_registry)
	{
		num_sdk_msgs::SaveDataRate d;
		d.data_product = entry.first;
		d.save_rate_hz = entry.second.first;
		res.data_products.push_back(d);
	}
	return true;
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
