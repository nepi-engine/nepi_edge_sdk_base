#include <sys/stat.h>

#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "node_3dx.h"
#include "trigger_interface.h"
#include "save_data_interface.h"

#define BOOL_TO_ENABLED(x)	((x)==true)? "enabled" : "disabled"

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
	_intensity_enabled{"intensity_enabled", false, this},
	_intensity_control{"intensity_control", 0.0f, this},
	_img_0_name{"img_0_name", "img_0", this},
	_img_1_name{"img_1_name", "img_1", this},
	_alt_img_name{"alt_img_name", "alt", this}
{
	_save_data_if = new SaveDataInterface(this, &n, &n_priv);
	_save_data_if->registerDataProduct("img_0");
	_save_data_if->registerDataProduct("img_1");
	_save_data_if->registerDataProduct("img_alt");

	// Load the sim mode files
	const std::string SIM_IMG_BASENAME = "/opt/numurus/ros/etc/" + getUnqualifiedName() + "/sim_img_";

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
	_status_pub = n_priv.advertise<num_sdk_msgs::Status3DX>("status_3dx", 3, boost::bind(&Node3DX::publishStatus, this));
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
	_intensity_enabled.retrieve();
	_intensity_control.retrieve();
	_img_0_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again
	_img_1_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again
	_alt_img_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again

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
	subscribers.push_back(n_priv.subscribe("set_intensity", 3, &Node3DX::setIntensityHandler, this));
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
void Node3DX::setRangeHandler(const num_sdk_msgs::Range3DX::ConstPtr &msg)
{
	// Range-check inputs
	if (msg->min_range < 0.0f ||
		msg->max_range > 1.0f ||
		msg->min_range > msg->max_range)
	{
		ROS_ERROR("%s received invalid range settings (%.3f,%.3f)", getUnqualifiedName().c_str(), msg->min_range, msg->max_range);
		return;
	}

	const bool updated_range = (msg->min_range != _min_range) || (msg->max_range != _max_range);

	if (true == updated_range)
	{
		_min_range = msg->min_range;
		_max_range = msg->max_range;
		ROS_DEBUG("%s updated range to (%.3f,%.3f)", getUnqualifiedName().c_str(), msg->min_range, msg->max_range);
		publishStatus();
	}
}

void Node3DX::setAngleHandler(const num_sdk_msgs::Angle3DX::ConstPtr &msg)
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

void Node3DX::setResolutionHandler(const num_sdk_msgs::AutoManualSelection3DX::ConstPtr &msg)
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

void Node3DX::setGainHandler(const num_sdk_msgs::AutoManualSelection3DX::ConstPtr &msg)
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

void Node3DX::setFilterHandler(const num_sdk_msgs::AutoManualSelection3DX::ConstPtr &msg)
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

void Node3DX::setIntensityHandler(const num_sdk_msgs::AutoManualSelection3DX::ConstPtr &msg)
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

void Node3DX::publishImage(int id, cv::Mat *img, sensor_msgs::CameraInfoPtr cinfo, ros::Time *tstamp, bool save_if_necessary)
{
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, *img).toImageMsg();

	publishImage(id, msg, cinfo, save_if_necessary);
}

void Node3DX::publishImage(int id, sensor_msgs::ImagePtr img, sensor_msgs::CameraInfoPtr cinfo, bool save_if_necessary)
{
	// TODO: This method is not threadsafe, but called by multiple threads in e.g., gs_multicam
	image_transport::CameraPublisher *publisher = nullptr;
	sensor_msgs::ImagePtr img_out = nullptr;
	switch (id)
	{
	case IMG_0:
		publisher = &img_0_pub;
		img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_0_sim_data).toImageMsg() : img;
		break;
	case IMG_1:
		publisher = &img_1_pub;
		img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_1_sim_data).toImageMsg() : img;
		break;
	case IMG_ALT:
		publisher = &img_alt_pub;
		img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_alt_sim_data).toImageMsg() : img;
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

void Node3DX::publishImage(int img_id, sensor_msgs::ImageConstPtr img, sensor_msgs::CameraInfoConstPtr cinfo, bool save_if_necessary)
{
	switch (img_id)
	{
	case IMG_0:
		{
			sensor_msgs::ImageConstPtr img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_0_sim_data).toImageMsg() : img;
			img_0_pub.publish(img_out, cinfo);
			if (true == save_if_necessary)
			{
				saveDataIfNecessary(img_id, img_out);
			}
		}
		break;
	case IMG_1:
		{
			sensor_msgs::ImageConstPtr img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_1_sim_data).toImageMsg() : img;
			img_1_pub.publish(img_out, cinfo);
			if (true == save_if_necessary)
			{
				saveDataIfNecessary(img_id, img_out);
			}
		}
		break;
	case IMG_ALT:
		{
			sensor_msgs::ImageConstPtr img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, ros_cam_color_encoding_name, img_alt_sim_data).toImageMsg() : img;
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

void Node3DX::saveDataIfNecessary(int img_id, sensor_msgs::ImageConstPtr img)
{
	if (false == _save_data_if->saveContinuousEnabled())
	{
		return; // Nothing else to do
	}

	std::string image_identifier;
	const std::string output_img_encoding = (img_id == IMG_ALT)? "mono8" : "bgr8";
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

	cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    //cv_ptr = cv_bridge::toCvShare(img, img->encoding);
    cv_ptr = cv_bridge::toCvShare(img, output_img_encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Capture the timestamp in a good format for filenames
  const std::string display_name = _display_name;
	const std::string tstamp_str = _save_data_if->getTimestampString();
	// To change the permissions - opencv API doesn't seem to give us that control at creation
	static const mode_t mode = S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH; // 664

  // Create the filename - defer the extension so that we can adjust as necessary for save_raw
	std::stringstream qualified_filename;
  qualified_filename << _save_data_if->_save_data_dir << "/" << _save_data_if->getFilenamePrefix() <<
    									display_name << "_" << image_identifier << "_" << tstamp_str << ".png";
  //const std::string jpg_filename = qualified_filename_no_extension.str() + ".jpg";
	bool success = cv::imwrite( qualified_filename.str(),  cv_ptr->image ); // OpenCV uses extensions intelligently
	if (false == success)
	{
		ROS_ERROR_STREAM_THROTTLE(1, "Could not save " << qualified_filename.str());
	}
	chmod(qualified_filename.str().c_str(), mode);
}

void Node3DX::publishStatus()
{
	num_sdk_msgs::Status3DX msg;

	msg.display_name = _display_name;

	msg.pause_enable = _paused;

	msg.simulate_data = _simulated_data.getParam();

	msg.range.min_range = _min_range.getParam();
	msg.range.max_range = _max_range.getParam();

	msg.angle.angle_offset = _angle_offset.getParam();
	msg.angle.total_angle = _total_angle.getParam();

	msg.resolution_settings.enabled = _manual_resolution_enabled.getParam();
	msg.resolution_settings.adjustment = _manual_resolution.getParam();

	msg.gain_settings.enabled = _gain_enabled.getParam();
	msg.gain_settings.adjustment = _gain.getParam();

	msg.filter_settings.enabled = _filter_enabled.getParam();
	msg.filter_settings.adjustment = _filter_control.getParam();

	msg.intensity_settings.enabled = _intensity_enabled.getParam();
	msg.intensity_settings.adjustment = _intensity_control.getParam();

	_status_pub.publish(msg);
}

void Node3DX::run()
{
	// Do init() if it hasn't already been done
	if (false == _initialized)
	{
		init();
	}

	// Set up periodic publishing of the 3DX status
	const ros::Duration status_pub_period(2.0);
	ros::Time next_status_pub = ros::Time::now() + status_pub_period;
	ROS_ERROR_STREAM("Debugging: now = " << ros::Time::now() << ", first status pub = " << next_status_pub);
  // Spin at the current rate
	while (ros::ok())
  {
    ros::Rate r(current_rate_hz);
		const ros::Time now = ros::Time::now();
		if (now >= next_status_pub)
		{
			ROS_ERROR("Debugging: About to publish periodic status");
			publishStatus();
			next_status_pub = now + status_pub_period;
		}
	  ros::spinOnce();
    r.sleep();
  }
}

} // namespace numurus
