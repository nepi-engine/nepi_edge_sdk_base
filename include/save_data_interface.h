#ifndef _SAVE_DATA_INTERFACE_H
#define _SAVE_DATA_INTERFACE_H

#include <string>
#include <unordered_map>
#include <utility>
#include <array>
#include <unistd.h>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <nepi_ros_interfaces/SaveData.h>
#include <nepi_ros_interfaces/SaveDataRate.h>
#include <nepi_ros_interfaces/DataProductQuery.h>

#include <sdk_interface.h>
#include <sdk_node.h>

namespace Numurus
{

/**
 * @brief      Provides the consistent ROS interface for any node that can save data to disk
 */
class SaveDataInterface : public SDKInterface
{
public:
	/**
	 * @brief      Standard constructor
	 *
	 * @see {SDKInterface}
	 *
	 * @param      parent          The parent
	 * @param      parent_pub_nh   The parent pub nh
	 * @param      parent_priv_nh  The parent priv nh
	 */
	SaveDataInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh);

	SaveDataInterface() = delete;

	virtual ~SaveDataInterface();

	// Inherited from SDKInterface
	void retrieveParams() override;
	void initPublishers() override;
	void initSubscribers() override;
	void initServices() override;

	inline bool saveContinuousEnabled() {return (_save_continuous == true);}
	inline bool saveRawEnabled() {return (_save_continuous == true && _save_raw == true);}
	inline const std::string getFilenamePrefix() {return _filename_prefix;}

	void registerDataProduct(const std::string product_name, double save_rate_hz = 1.0, double max_save_rate_hz = 100.0);
	bool dataProductShouldSave(const std::string product_name, ros::Time data_time = ros::Time::now());
	bool calibrationShouldSave();

	static std::string getTimestampString(const ros::Time &time = ros::Time::now());

	const std::string getFullPathFilename(const std::string &timestamp_string, const std::string &identifier, const std::string &extension);

	/**
	 * Non-user-configurable base path
	 */
	const std::string _save_data_dir = "/home/numurus_user/data";
	const uid_t _data_uid = 1001; // numurus_user
	const gid_t _data_gid = 1001; // numurus_user

protected:
	/**
	 * Param-server-backed bool indicating whether the parent node should save data continuously
	 */
	SDKNode::NodeParam<bool> _save_continuous;

	/**
	 * Param-server-backed bool indicating whether the parent node should save raw data (where 'raw' is node-defined)
	 */
	SDKNode::NodeParam<bool> _save_raw;

	/**
	 * User-configurable identifying prefix
	 */
	std::string _filename_prefix;

	/**
	 * ROS publisher for the save_status topic
	 */
	ros::Publisher _save_status_pub;

	// Form for the array is <rate_hz, next_save_time_s, max_rate_hz>
	typedef std::array<double, 3> data_product_registry_entry_t;
	std::unordered_map<std::string, data_product_registry_entry_t> data_product_registry;

	bool _needs_save_calibration = false;

	/**
	 * @brief      Callback for save_data topic subscriptiion
	 *
	 * @param[in]  msg   Indicates the save_data settings
	 */
	void saveDataHandler(const nepi_ros_interfaces::SaveData::ConstPtr &msg);

	/**
	 * @brief      Callback for the save_data_prefix topic subscription
	 *
	 * @param[in]  msg   Indicates the save_data_prefix settings
	 */
	void saveDataPrefixHandler(const std_msgs::String::ConstPtr &msg);

	void saveDataRateHandler(const nepi_ros_interfaces::SaveDataRate::ConstPtr &msg);

	bool queryDataProductsHandler(nepi_ros_interfaces::DataProductQuery::Request &req, nepi_ros_interfaces::DataProductQuery::Response &res);

	/**
	 * @brief      Publish the save_data settings on the save_status topic
	 */
	void publishSaveStatus();

}; // class SaveDataInterface

} // namespace Numurus

#endif // _SAVE_DATA_INTERFACE_H
