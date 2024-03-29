/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <sys/stat.h>

#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "node_3dx.h"
#include "trigger_interface.h"
#include "save_data_interface.h"

#define BOOL_TO_ENABLED(x)	((x)==true)? "enabled" : "disabled"

#define SAVE_DATA_BUFFER_SIZE	25 // 25 data entries max before some start getting dropped
#define SAVE_DATA_THREAD_COUNT	4

namespace Numurus
{

Node3DX::Node3DX():
	img_trans{n_priv}, // Image topics are published in the node-specific namespace
	_simulated_data{"simulated_data", false, this},
	_min_range{"min_range", 0.0f, this},
	_max_range{"max_range", 1.0f, this},
	_angle_offset{"angle_offset", 0.0f, this},
	_total_angle{"total_angle", 1.0f, this},
	_manual_resolution_enabled{"manual_resolution_enabled", true, this},
	_manual_resolution{"manual_resolution", 1.0f, this},
	_gain_enabled{"gain_enabled", true, this},
	_gain{"gain", 1.0f, this},
	_filter_enabled{"filter_enabled", false, this},
	_filter_control{"filter_control", 0.0f, this},
	_enhancement_enabled{"enhancement_enabled", false, this},
	_enhancement_control{"enhancement_control", 0.0f, this},
	_intensity_enabled{"intensity_enabled", false, this},
	_intensity_control{"intensity_control", 0.0f, this},
	_img_0_name{"img_0_name", "img_0", this},
	_img_1_name{"img_1_name", "img_1", this},
	_alt_img_name{"alt_img_name", "alt", this},
	_3d_data_target_frame{"data_3d_target_frame", "3dx_center_frame", this},
	stitched_cloud_enabled{"stitched_cloud/enabled", false, this},
	stitched_cloud_resolution_m{"stitched_cloud/resolution_m", 0.01f, this},
	stitched_cloud_pub_rate_hz{"stitched_cloud/pub_rate_hz", 5.0f, this},
	stitched_cloud_auto_range_enabled{"stitched_cloud/auto_range_enabled", false, this},
	stitched_cloud_max_manual_range_m{"stitched_cloud/max_manual_range_m", 4.0f, this},
	stitched_cloud_size_lim_mb{"stitched_cloud/size_lim_mb", 512, this},
	stitched_cloud_max_save_rate_hz{"stitched_cloud/max_save_rate_hz", 0.5f, this},
	save_data_buffer{SAVE_DATA_BUFFER_SIZE}
{
	nepi_ros_interfaces::Status3DX status_3dx;
	status_flags.resize(status_3dx.flags.max_size(), false); // std::vector sized from boost::array
	status_flags[nepi_ros_interfaces::Status3DX::DEVICE_STARTING] = true;

	_save_data_if = new SaveDataInterface(this, &n, &n_priv);
	_save_data_if->registerDataProduct("img_0");
	_save_data_if->registerDataProduct("img_1");
	_save_data_if->registerDataProduct("img_alt");

	// Load the sim mode files
	const std::string SIM_IMG_BASENAME = "/opt/nepi/ros/etc/" + getUnqualifiedName() + "/sim_img_";

	const std::string IMG_0_SIM_FILENAME = SIM_IMG_BASENAME + "0.png";
	loadSimData(IMG_0_SIM_FILENAME, &img_0_sim_data);

	const std::string IMG_1_SIM_FILENAME = SIM_IMG_BASENAME + "1.png";
	loadSimData(IMG_1_SIM_FILENAME, &img_1_sim_data);

	const std::string IMG_ALT_SIM_FILENAME = SIM_IMG_BASENAME + "alt.png";
	loadSimData(IMG_ALT_SIM_FILENAME, &img_alt_sim_data);

	ros_cam_color_encoding_name = "rgb8"; // Default
}

Node3DX::~Node3DX()
{
	delete _save_data_if;
	_save_data_if = nullptr;

	terminate_save_data_threads = true;
	for (std::thread *t : save_data_threads)
	{
		t->join();
		delete t;
	}
}

void Node3DX::loadSimData(std::string filename, cv::Mat *out_mat)
{
	*out_mat = cv::imread(filename);
	if (NULL == out_mat->data)
	{
		ROS_ERROR("Unable to load sim data from file %s", filename.c_str());
	}
}

void Node3DX::initPublishers()
{
	// Call the base method
	SDKNode::initPublishers();

	// The image names are NodeParam types, so we need to obtain them first and use a conversion operator to their data type
	_img_0_name.retrieve();
	img_0_pub = img_trans.advertiseCamera((std::string)_img_0_name + "/image_raw", 1);
	_img_1_name.retrieve();
	img_1_pub = img_trans.advertiseCamera((std::string)_img_1_name + "/image_raw", 1);
	_alt_img_name.retrieve();
	img_alt_pub = img_trans.advertiseCamera((std::string)_alt_img_name + "/image_raw", 1);

	// Advertise the status_3dx topic, using the overload form that provides a callback on new subscriber connection.
	// Want to always send a status update whenever a subscriber connects.
	//_status_pub = n_priv.advertise<nepi_ros_interfaces::Status3DX>("status_3dx", 3, boost::bind(&Node3DX::publishStatus, this));
	_status_pub = n_priv.advertise<nepi_ros_interfaces::Status3DX>("status_3dx", 3, true);
	_transformed_pointcloud_pub = n_priv.advertise<sensor_msgs::PointCloud2>("pointcloud_3dx", 3);
}

void Node3DX::retrieveParams()
{
	// Call the base method
	SDKNode::retrieveParams();

	// Grab the node_3dx parameters
	_simulated_data.retrieve();
	_min_range.retrieve();
	_max_range.retrieve();
	_angle_offset.retrieve();
	_total_angle.retrieve();
	_manual_resolution_enabled.retrieve();
	_manual_resolution.retrieve();
	_gain_enabled.retrieve();
	_gain.retrieve();
	_filter_enabled.retrieve();
	_filter_control.retrieve();
	_enhancement_enabled.retrieve();
	_enhancement_control.retrieve();
	_intensity_enabled.retrieve();
	_intensity_control.retrieve();
	_img_0_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again
	_img_1_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again
	_alt_img_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again
	_3d_data_target_frame.retrieve();
	stitched_cloud_enabled.retrieve();
	stitched_cloud_resolution_m.retrieve();
	stitched_cloud_pub_rate_hz.retrieve();
	stitched_cloud_auto_range_enabled.retrieve();
	stitched_cloud_max_manual_range_m.retrieve();
	stitched_cloud_size_lim_mb.retrieve();
	stitched_cloud_max_save_rate_hz.retrieve();

	// Image transport parameters are ROS "dynamic_params", so don't need to be retrieved manually

	// Make sure to retrieve interface params, too
	if (nullptr != _trig_if)
	{
		_trig_if->retrieveParams();
	}
	if (nullptr != _save_data_if)
	{
		_save_data_if->retrieveParams();
	}

	// Send a status update whenever we init params
	publishStatus();

	// This is as good a place as any to start the image saver threads
	terminate_save_data_threads = false;
	for (size_t i = 0; i < SAVE_DATA_THREAD_COUNT; ++i)
	{
		save_data_threads.push_back(new std::thread(&Node3DX::saveDataRun, this));
	}
}

void Node3DX::initSubscribers()
{
	// Call the base method
	SDKNode::initSubscribers();

	// Now subscribe to the set of global nd controls
	// These versions are in the public namespace so that we can support global commands
	subscribers.push_back(n.subscribe("pause_enable", 3, &Node3DX::pauseEnableHandler, this));
	subscribers.push_back(n.subscribe("simulate_data", 3, &Node3DX::simulateDataHandler, this));

	// Now subscribe to the private namespace versions
	subscribers.push_back(n_priv.subscribe("pause_enable", 3, &Node3DX::pauseEnableHandler, this));
	subscribers.push_back(n_priv.subscribe("simulate_data", 3, &Node3DX::simulateDataHandler, this));

	// Also in the private namespace are the various generic "tweaks"
	subscribers.push_back(n_priv.subscribe("set_range", 3, &Node3DX::setRangeHandler, this));
	subscribers.push_back(n_priv.subscribe("set_angle", 3, &Node3DX::setAngleHandler, this));
	subscribers.push_back(n_priv.subscribe("set_resolution", 3, &Node3DX::setResolutionHandler, this));
	subscribers.push_back(n_priv.subscribe("set_gain", 3, &Node3DX::setGainHandler, this));
	subscribers.push_back(n_priv.subscribe("set_filter", 3, &Node3DX::setFilterHandler, this));
	subscribers.push_back(n_priv.subscribe("set_enhancement", 3, &Node3DX::setEnhancementHandler, this));
	subscribers.push_back(n_priv.subscribe("set_intensity", 3, &Node3DX::setIntensityHandler, this));
	subscribers.push_back(n_priv.subscribe("clear_status_flags", 3, &Node3DX::clearStatusFlagsHandler, this));

	subscribers.push_back(n_priv.subscribe("set_pointcloud_target_frame", 3, &Node3DX::set3dDataTargetFrameHandler, this));
	subscribers.push_back(n_priv.subscribe("enable_stitched_cloud", 3, &Node3DX::enableStitchedCloudHandler, this));
}

void Node3DX::pauseEnableHandler(const std_msgs::Bool::ConstPtr &msg)
{
	if (msg->data != _paused)
	{
		_paused = msg->data;
		if (nullptr != _trig_if)
		{
			_trig_if->setTrigEnabled(!_paused); // awkward parity change
		}
		else
		{
			ROS_ERROR("%s: The TriggerInterface was not properly initialized", getUnqualifiedName().c_str());
		}
		ROS_DEBUG("%s pause settings updated (pause=%s)", getUnqualifiedName().c_str(), BOOL_TO_ENABLED(_paused));
		publishStatus();
	}
}

void Node3DX::simulateDataHandler(const std_msgs::Bool::ConstPtr &msg)
{
	const bool simulated_data_updated = (msg->data != _simulated_data);
	if (true == simulated_data_updated)
	{
		_simulated_data = msg->data;
		ROS_DEBUG("%s:  simulation mode is now %s", getUnqualifiedName().c_str(), BOOL_TO_ENABLED(_simulated_data));
		publishStatus();
	}
}

// Node-specific subscription callbacks. Concrete instances should define what actions these take,
// though we provide a very basic private member setter implementation in this baseclass
void Node3DX::setRangeHandler(const nepi_ros_interfaces::RangeWindow::ConstPtr &msg)
{
	// Range-check inputs
	if (msg->start_range < 0.0f ||
		msg->stop_range > 1.0f ||
		msg->start_range > msg->stop_range)
	{
		ROS_ERROR("%s received invalid range settings (%.3f,%.3f)", getUnqualifiedName().c_str(), msg->start_range, msg->stop_range);
		return;
	}

	const bool updated_range = (msg->start_range != _min_range) || (msg->stop_range != _max_range);

	if (true == updated_range)
	{
		_min_range = msg->start_range;
		_max_range = msg->stop_range;
		ROS_DEBUG("%s updated range to (%.3f,%.3f)", getUnqualifiedName().c_str(), msg->start_range, msg->stop_range);
		publishStatus();
	}
}

void Node3DX::setAngleHandler(const nepi_ros_interfaces::Angle3DX::ConstPtr &msg)
{
	// Generic bounds checking for "angle"
	if (msg->angle_offset < 0.0f ||
		  msg->total_angle < 0.0f  ||
		  msg->angle_offset > 1.0f ||
		  msg->total_angle > 1.0f)
	{
		ROS_ERROR("%s received invalid angle settings (%.3f,%.3f)", getUnqualifiedName().c_str(), msg->angle_offset, msg->total_angle);
		return;
	}

	const bool updated_angle = (msg->angle_offset != _angle_offset) || (msg->total_angle != _total_angle);
	if (true == updated_angle)
	{
		_angle_offset = msg->angle_offset;
		_total_angle = msg->total_angle;
		ROS_DEBUG("%s updated angle to (%.3f,%.3f)", getUnqualifiedName().c_str(), msg->angle_offset, msg->total_angle);
		publishStatus();
	}
}

void Node3DX::setResolutionHandler(const nepi_ros_interfaces::AutoManualSelection3DX::ConstPtr &msg)
{
	if (false == autoManualMsgIsValid(msg))
	{
		ROS_ERROR("%s received invalid resolution settings (adjustment = %.3f)", getUnqualifiedName().c_str(), msg->adjustment);
		return;
	}

	const bool updated_resolution = (msg->enabled != _manual_resolution_enabled) || (msg->adjustment != _manual_resolution);
	if (true == updated_resolution)
	{
		_manual_resolution_enabled = msg->enabled;
		_manual_resolution = msg->adjustment;
		ROS_DEBUG("%s updated resolution to %s:%.3f", getUnqualifiedName().c_str(),
			(_manual_resolution_enabled)? "manual":"auto",
			(_manual_resolution_enabled)? _manual_resolution : 0.0f);
		publishStatus();
	}
}

void Node3DX::setGainHandler(const nepi_ros_interfaces::AutoManualSelection3DX::ConstPtr &msg)
{
	if (false == autoManualMsgIsValid(msg))
	{
		ROS_ERROR("%s received invalid gain settings (adjustment = %.3f)", getUnqualifiedName().c_str(), msg->adjustment);
		return;
	}

	const bool updated_gain = (msg->enabled != _gain_enabled) || (msg->adjustment != _gain);
	if (true == updated_gain)
	{
		_gain_enabled = msg->enabled;
		_gain = msg->adjustment;
		ROS_DEBUG("%s updated gain to %s:%.3f", getUnqualifiedName().c_str(),
			(_gain_enabled)? "enabled":"disabled",
			(_gain_enabled)? _gain : 0.0f);
		publishStatus();
	}
}

void Node3DX::setFilterHandler(const nepi_ros_interfaces::AutoManualSelection3DX::ConstPtr &msg)
{
	if (false == autoManualMsgIsValid(msg))
	{
		ROS_ERROR("%s received invalid filter settings (adjustment = %.3f)", getUnqualifiedName().c_str(), msg->adjustment);
		return;
	}

	const bool updated_filter = (msg->enabled != _filter_enabled) || (msg->adjustment != _filter_control);
	if (true == updated_filter)
	{
		_filter_enabled = msg->enabled;
		_filter_control = msg->adjustment;
		ROS_DEBUG("%s updated filter settings to %s:%.3f", getUnqualifiedName().c_str(),
			(_filter_enabled)? "enabled":"disabled",
			(_filter_enabled)? _filter_control : 0.0f);
		publishStatus();
	}
}

void Node3DX::setEnhancementHandler(const nepi_ros_interfaces::AutoManualSelection3DX::ConstPtr &msg)
{
	if (false == autoManualMsgIsValid(msg))
	{
		ROS_ERROR("%s received invalid enhancement settings (adjustment = %.3f)", getUnqualifiedName().c_str(), msg->adjustment);
		return;
	}

	const bool updated_enhancement = (msg->enabled != _enhancement_enabled) || (msg->adjustment != _enhancement_control);
	if (true == updated_enhancement)
	{
		_enhancement_enabled = msg->enabled;
		_enhancement_control = msg->adjustment;
		ROS_DEBUG("%s updated enhancement settings to %s:%.3f", getUnqualifiedName().c_str(),
			(_enhancement_enabled)? "enabled":"disabled",
			(_enhancement_enabled)? _enhancement_control : 0.0f);
		publishStatus();
	}
}

void Node3DX::setIntensityHandler(const nepi_ros_interfaces::AutoManualSelection3DX::ConstPtr &msg)
{
	if (false == autoManualMsgIsValid(msg))
	{
		ROS_ERROR("%s received invalid intensity settings (adjustment = %.3f)", getUnqualifiedName().c_str(), msg->adjustment);
		return;
	}

	const bool updated_intensity = (msg->enabled != _intensity_enabled) || (msg->adjustment != _intensity_control);
	if (true == updated_intensity)
	{
		_intensity_enabled = msg->enabled;
		_intensity_control = msg->adjustment;
		ROS_DEBUG("%s updated intensity settings to %s:%.3f", getUnqualifiedName().c_str(),
			(_intensity_enabled)? "enabled":"disabled",
			(_intensity_enabled)? _intensity_control : 0.0f);
		publishStatus();
	}
}

void Node3DX::clearStatusFlagsHandler(const std_msgs::Empty::ConstPtr &msg)
{
	ROS_INFO("%s clearing status flags by request", getUnqualifiedName().c_str());

	nepi_ros_interfaces::Status3DX status_3dx;
	for (size_t i = 0; i < status_3dx.flags.max_size(); ++i)
	{
		status_flags[i] = false;
	}
}

void Node3DX::publishImage(int id, cv::Mat *img, sensor_msgs::CameraInfoPtr cinfo, ros::Time *tstamp, bool save_if_necessary, bool override_simulated_data)
{
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, *img).toImageMsg();

	publishImage(id, msg, cinfo, save_if_necessary, override_simulated_data);
}

void Node3DX::publishImage(int id, sensor_msgs::ImagePtr img, sensor_msgs::CameraInfoPtr cinfo, bool save_if_necessary, bool override_simulated_data)
{
	// TODO: This method is not threadsafe, but called by multiple threads in e.g., gs_multicam
	image_transport::CameraPublisher *publisher = nullptr;
	sensor_msgs::ImagePtr img_out = nullptr;

	const bool simulate_data = (true == override_simulated_data)? false : _simulated_data;
	switch (id)
	{
	case IMG_0:
		publisher = &img_0_pub;
		img_out = (true == simulate_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_0_sim_data).toImageMsg() : img;
		break;
	case IMG_1:
		publisher = &img_1_pub;
		img_out = (true == simulate_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_1_sim_data).toImageMsg() : img;
		break;
	case IMG_ALT:
		publisher = &img_alt_pub;
		img_out = (true == simulate_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_alt_sim_data).toImageMsg() : img;
		break;
	default:
		ROS_ERROR("%s: Request to publish unknown image id (%d)", getUnqualifiedName().c_str(), id);
		return;
	}

	publisher->publish(img_out, cinfo);

	if (true == save_if_necessary)
	{
		saveDataIfNecessary(id, img_out);
	}
}

void Node3DX::publishImage(int img_id, sensor_msgs::ImageConstPtr img, sensor_msgs::CameraInfoConstPtr cinfo, bool save_if_necessary, bool override_simulated_data)
{
	const bool simulate_data = (true == override_simulated_data)? false : _simulated_data;
	switch (img_id)
	{
	case IMG_0:
		{
			sensor_msgs::ImageConstPtr img_out = (true == simulate_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_0_sim_data).toImageMsg() : img;
			img_0_pub.publish(img_out, cinfo);
			if (true == save_if_necessary)
			{
				saveDataIfNecessary(img_id, img_out);
			}
		}
		break;
	case IMG_1:
		{
			sensor_msgs::ImageConstPtr img_out = (true == simulate_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_1_sim_data).toImageMsg() : img;
			img_1_pub.publish(img_out, cinfo);
			if (true == save_if_necessary)
			{
				saveDataIfNecessary(img_id, img_out);
			}
		}
		break;
	case IMG_ALT:
		{
			sensor_msgs::ImageConstPtr img_out = (true == simulate_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_alt_sim_data).toImageMsg() : img;
			img_alt_pub.publish(img_out, cinfo);
			if (true == save_if_necessary)
			{
				saveDataIfNecessary(img_id, img_out);
			}
		}
		break;
	default:
		ROS_ERROR("%s: Request to publish unknown image id (%d)", getUnqualifiedName().c_str(), img_id);
		return;
	}
}

bool Node3DX::transformCloudAndRepublish(const sensor_msgs::PointCloud2::ConstPtr &input, sensor_msgs::PointCloud2Ptr &transformed_cloud)
{
	const std::string target_frame = _3d_data_target_frame;
	//ros::Time start_time = ros::Time::now(); // Testing only
	if (false == pcl_ros::transformPointCloud(target_frame, *input, *transformed_cloud, _transform_listener))
	{
		ROS_ERROR("Failed to transform point cloud from frame_id %s to frame_id: %s", input->header.frame_id.c_str(), target_frame.c_str());
		return false;
	}
	//ros::Time stop_time = ros::Time::now(); // Testing only
	//ROS_WARN("Debug: pointcloud conversion took %f secs", (stop_time - start_time).toSec()); // Testing only

	_transformed_pointcloud_pub.publish(transformed_cloud);

	// TODO: Use the transformed point cloud to generate a transformed depth image
	// Take inspiration from here:
	// https://answers.ros.org/question/304857/converting-a-xyz-point-cloud-to-a-depth-image/
	// or here:
	// https://github.com/tu-darmstadt-ros-pkg/hector_cloud_image_proc/blob/master/pcl_to_cv_proc/include/pcl_to_cv_proc/to_cv_depth_img.h

	return true;
}

void Node3DX::set3dDataTargetFrameHandler(const std_msgs::String::ConstPtr &msg)
{
	// TODO: Check that this is a valid frame?
	_3d_data_target_frame = msg->data;
}

void Node3DX::enableStitchedCloudHandler(const std_msgs::Bool::ConstPtr &msg)
{
	stitched_cloud_enabled = msg->data;
}

void Node3DX::saveDataIfNecessary(int img_id, sensor_msgs::ImageConstPtr img)
{
	if (false == _save_data_if->saveContinuousEnabled())
	{
		return; // Nothing else to do
	}

	std::string image_identifier;
	// Use BGR8 unless this is the alt image, which can optionally be grayscale or color -- just use whatever is defined.
	const std::string output_img_encoding = (img_id == IMG_ALT)?
	 	img->encoding : "bgr8";
	switch(img_id)
	{
	case IMG_0:
		image_identifier = "img_0";
		break;
	case IMG_1:
		image_identifier = "img_1";
		break;
	case IMG_ALT:
		image_identifier = "img_alt";
		break;
	default:
		ROS_ERROR("%s: Request to save for unknown image id (%d)", getUnqualifiedName().c_str(), img_id);
	}

	// Check that it is time to save this data product
	if (false == _save_data_if->dataProductShouldSave(image_identifier))
	{
		// Not time yet -- just return silently
		return;
	}

	if (true == _save_data_if->calibrationShouldSave())
	{
		saveSensorCalibration();
	}

	 // Capture the timestamp in a good format for filenames
  const std::string display_name = _display_name;
	// To change the permissions - opencv API doesn't seem to give us that control at creation

  // Create the filename - defer the extension so that we can adjust as necessary for save_raw
	const std::string qualified_filename = _save_data_if->getFullPathFilename(_save_data_if->getTimestampString(img->header.stamp), display_name + "_" + image_identifier, "png");
	//const std::string jpg_filename = qualified_filename_no_extension.str() + ".jpg";
	SaveDataStruct save_data(img, qualified_filename, output_img_encoding);
	// Critical section
	{
		std::lock_guard<std::mutex> l(save_data_buffer_mutex);
		save_data_buffer.push_back(save_data);
	}
}

void Node3DX::publishStatus()
{
	nepi_ros_interfaces::Status3DX msg;

	msg.display_name = _display_name;

	msg.pause_enable = _paused;

	msg.simulate_data = _simulated_data;

	msg.range.start_range = _min_range;
	msg.range.stop_range = _max_range;

	msg.angle.angle_offset = _angle_offset;
	msg.angle.total_angle = _total_angle;

	msg.resolution_settings.enabled = _manual_resolution_enabled;
	msg.resolution_settings.adjustment = _manual_resolution;

	msg.gain_settings.enabled = _gain_enabled;
	msg.gain_settings.adjustment = _gain;

	msg.filter_settings.enabled = _filter_enabled;
	msg.filter_settings.adjustment = _filter_control;

	msg.enhancement_settings.enabled = _enhancement_enabled;
	msg.enhancement_settings.adjustment = _enhancement_control;

	msg.intensity_settings.enabled = _intensity_enabled;
	msg.intensity_settings.adjustment = _intensity_control;

	msg.frame_3d = _3d_data_target_frame;

	msg.stitched_cloud_enabled = stitched_cloud_enabled;

	for (size_t i = 0; i < msg.flags.max_size(); ++i)
	{
		msg.flags[i] = status_flags[i]; // Must assign std::vector to boost::array element-by-element
	}

	_status_pub.publish(msg);
}

void Node3DX::saveDataRun()
{
	while (false == terminate_save_data_threads)
	{
		ros::Duration(0.02).sleep(); // Don't let this thread starve everything else
		SaveDataStruct save_data;
		// Critical section
		bool buffer_full;
		bool skip_redundant_save = false;
		{
			std::lock_guard<std::mutex> l(save_data_buffer_mutex);
			if (false == save_data_buffer.empty())
			{
				buffer_full = (save_data_buffer.size() == SAVE_DATA_BUFFER_SIZE);
				save_data = save_data_buffer[0];
				save_data_buffer.pop_front();

				// If full, check for redundant files (e.g., stitched pointclouds) to skip as an efficiency speedup
				if (true == buffer_full)
				{
					for (auto &more_recent_saved_data : save_data_buffer)
					{
						if (more_recent_saved_data.qualified_filename == save_data.qualified_filename)
						{
							skip_redundant_save = true;
							break;
						}
					}
				}
			}
			else
			{
				continue;
			}
		}

		if (true == buffer_full) // Do this warning outside mutex protection for better efficiency
		{
			ROS_WARN_THROTTLE(3, "Data saving is not keeping up -- oldest data will be discarded");
		}

		if (true == skip_redundant_save)
		{
			ROS_WARN_THROTTLE(3, "Skipping redundant %s because there is a more recent version", save_data.qualified_filename.c_str());
			continue;
		}

		bool success = false;
		if (save_data.data_type == SaveDataStruct::SAVE_DATA_TYPE_IMG_PTR)
		{
			// Now construct the CV image
			cv_bridge::CvImageConstPtr cv_ptr;
			try
		  {
		    //cv_ptr = cv_bridge::toCvShare(img, img->encoding);
		    cv_ptr = cv_bridge::toCvShare(save_data.img_ptr, save_data.output_img_encoding);
		  }
		  catch (cv_bridge::Exception& e)
		  {
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    continue;
		  }

			success = cv::imwrite( save_data.qualified_filename,  cv_ptr->image ); // OpenCV uses extensions intelligently
		}
		else if (save_data.data_type == SaveDataStruct::SAVE_DATA_TYPE_RAW_MAT)
		{
			success = cv::imwrite( save_data.qualified_filename,  save_data.raw_mat ); // OpenCV uses extensions intelligently
		}
		else if (save_data.data_type == SaveDataStruct::SAVE_DATA_TYPE_CLOUD_PTR)
		{
			pcl::PointCloud<pcl::PointXYZRGB> cloud;
			pcl::fromROSMsg(*(save_data.cloud_ptr), cloud);

			try
			{
				if (0 == pcl::io::savePCDFileBinaryCompressed(save_data.qualified_filename, cloud))
				{
					//pcl::io::savePCDFile(qualified_filename, cloud); // ASCII Version
					success = true;
				}
			}
			catch (...) // savePCD will throw an exception for an empty pointcloud (and probably other stuff, too)
			{
				ROS_ERROR("Failed to save pointcloud");
			}
		}
		else
		{
			ROS_ERROR("Unexpected data type for data to save");
		}

		if (false == success)
		{
			ROS_ERROR("Unable to save %s", save_data.qualified_filename.c_str());
			continue;
		}

		// And make sure it has proper permissions
		static const mode_t mode = S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH; // 664
		chmod(save_data.qualified_filename.c_str(), mode);

		// Finally, sync all filesystems before returning
		int fd = open(save_data.qualified_filename.c_str(), O_RDONLY);
		if (-1 == fd)
		{
			ROS_ERROR_THROTTLE(1, "Not able to open the new data file for fsync()");
		}
		else
		{
			if (-1 ==fsync(fd))
			{
				ROS_ERROR_THROTTLE(1, "Not able to fsync() the new data file (errno = %d)", errno);
			}
			close(fd);
		}
	}
}

void Node3DX::run()
{
	// Do init() if it hasn't already been done
	if (false == _initialized)
	{
		init();
	}

	status_flags[nepi_ros_interfaces::Status3DX::DEVICE_STARTING] = false; // Clear the starting flag

	// Set up periodic publishing of the 3DX status
	//const ros::Duration status_pub_period(2.0);
	//ros::Time next_status_pub = ros::Time::now() + status_pub_period;
	//ROS_ERROR_STREAM("Debugging: now = " << ros::Time::now() << ", first status pub = " << next_status_pub);
  // Spin at the current rate
	while (ros::ok())
  {
		/*
    ros::Rate r(current_rate_hz);
		const ros::Time now = ros::Time::now();
		if (now >= next_status_pub)
		{
			//ROS_ERROR("Debugging: About to publish periodic status");
			publishStatus();
			next_status_pub = now + status_pub_period;
		}
	  ros::spinOnce();
    r.sleep();
		*/
		ros::spin();
  }
}

} // namespace numurus
