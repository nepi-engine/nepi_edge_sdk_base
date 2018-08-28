#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "nd_node.h"
#include "trigger_interface.h"

#define BOOL_TO_ENABLED(x)	((x)==true)? "enabled" : "disabled"

namespace Numurus
{

static void loadSimData(std::string filename, cv::Mat *out_mat)
{
	*out_mat = cv::imread(filename);
	if (NULL == out_mat->data)
	{
		ROS_ERROR("Unable to load sim data from file %s", filename.c_str());
	}
}

NDNode::NDNode():
	img_trans{n_priv}, // Image topics are published in the node-specific namespace
	_simulated_data{"simulated_data", false, this},
	_save_continuous{"save_data_continuous", false, this},
	_save_raw{"save_data_raw", false, this},
	_min_range{"min_range", 0.0f, this},
	_max_range{"max_range", 0.0f, this},
	_angle_offset{"angle_offset", 0.0f, this},
	_total_angle{"total_angle", 0.0f, this},
	_manual_resolution_enabled{"manual_resolution_enabled", true, this},
	_manual_resolution{"manual_resolution", 1.0f, this},
	_gain_enabled{"gain_enabled", true, this},
	_gain{"gain", 1.0f, this},
	_filter_enabled{"filter_enabled", true, this},
	_filter_control{"filter_control", 1.0f, this}
{
	// Load the sim mode files
	const std::string SIM_IMG_BASENAME = "/opt/numurus/ros/etc/" + getName() + "/sim_img_";
	
	const std::string IMG_0_SIM_FILENAME = SIM_IMG_BASENAME + "0.jpg";
	loadSimData(IMG_0_SIM_FILENAME, &img_0_sim_data);

	const std::string IMG_1_SIM_FILENAME = SIM_IMG_BASENAME + "1.jpg";
	loadSimData(IMG_1_SIM_FILENAME, &img_1_sim_data);

	const std::string IMG_ALT_SIM_FILENAME = SIM_IMG_BASENAME + "alt.jpg";
	loadSimData(IMG_ALT_SIM_FILENAME, &img_alt_sim_data);
}

void NDNode::initPublishers()
{
	// Call the base method
	SDKNode::initPublishers();

	img_0_pub = img_trans.advertise("img_0", 1);
	img_1_pub = img_trans.advertise("img_1", 1);
	img_alt_pub = img_trans.advertise("img_alt", 1);

	// Advertise the nd_status topic, using the overload form that provides a callback on new subscriber connection.
	// Want to always send a status update whenever a subscriber connects.
	_status_pub = n_priv.advertise<num_sdk_base::NDStatus>("nd_status", 3, boost::bind(&NDNode::publishStatus, this));	
}

void NDNode::retrieveParams()
{
	// Call the base method
	SDKNode::retrieveParams(); 
	
	// Grab the nd_node parameters
	_simulated_data.retrieve();
	_save_continuous.retrieve();
	_save_raw.retrieve();
	_min_range.retrieve();
	_max_range.retrieve();
	_angle_offset.retrieve();
	_total_angle.retrieve();
	_manual_resolution.retrieve();
	_manual_resolution.retrieve();
	_gain_enabled.retrieve();
	_gain.retrieve();
	_filter_enabled.retrieve();
	_filter_control.retrieve();

	// Image transport parameters are ROS "dynamic_params", so don't need to be retrieved manually

	// Make sure to retrieve trigger params, too
	_trig_if->retrieveParams();

	// Send a status update whenever we init params
	publishStatus();
}

void NDNode::initSubscribers()
{
	// Call the base method
	SDKNode::initSubscribers();

	// Now subscribe to the set of global nd controls
	// These versions are in the public namespace so that we can support global commands
	subscribers.push_back(n.subscribe("pause_enable", 3, &NDNode::pauseEnableHandler, this));
	subscribers.push_back(n.subscribe("save_data", 3, &NDNode::saveDataHandler, this));
	subscribers.push_back(n.subscribe("simulate_data", 3, &NDNode::simulateDataHandler, this));

	// Now subscribe to the private namespace versions
	subscribers.push_back(n_priv.subscribe("pause_enable", 3, &NDNode::pauseEnableHandler, this));
	subscribers.push_back(n_priv.subscribe("save_data", 3, &NDNode::saveDataHandler, this));
	subscribers.push_back(n_priv.subscribe("simulate_data", 3, &NDNode::simulateDataHandler, this));

	// Also in the private namespace are the various generic "tweaks"
	subscribers.push_back(n_priv.subscribe("set_range", 3, &NDNode::setRangeHandler, this));
	subscribers.push_back(n_priv.subscribe("set_angle", 3, &NDNode::setAngleHandler, this));
	subscribers.push_back(n_priv.subscribe("set_resolution", 3, &NDNode::setResolutionHandler, this));
	subscribers.push_back(n_priv.subscribe("set_gain", 3, &NDNode::setGainHandler, this));
	subscribers.push_back(n_priv.subscribe("set_filter", 3, &NDNode::setFilterHandler, this));
}

void NDNode::pauseEnableHandler(const std_msgs::Bool::ConstPtr &msg)
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
			ROS_ERROR("%s: The TriggerInterface was not properly initialized", getName().c_str());
		}
		ROS_DEBUG("%s pause settings updated (pause=%s)", getName().c_str(), BOOL_TO_ENABLED(_paused));
		publishStatus();
	}
}

void NDNode::saveDataHandler(const num_sdk_base::NDSaveData::ConstPtr &msg)
{
	const bool save_data_updated = (msg->save_continuous != _save_continuous) ||
								   (msg->save_raw != _save_raw);
	if (true == save_data_updated)
	{
		_save_continuous = msg->save_continuous;
		_save_raw = msg->save_raw;

		ROS_DEBUG("%s data save settings updated to (save_continuous=%s, save_raw=%s)", getName().c_str(),
				  BOOL_TO_ENABLED(_save_continuous), BOOL_TO_ENABLED(_save_raw));
		publishStatus();
	}
}

void NDNode::simulateDataHandler(const std_msgs::Bool::ConstPtr &msg)
{
	const bool simulated_data_updated = (msg->data != _simulated_data);
	if (true == simulated_data_updated)
	{
		_simulated_data = msg->data;
		ROS_DEBUG("%s:  simulation mode is now %s", getName().c_str(), BOOL_TO_ENABLED(_simulated_data));
		publishStatus();
	}
}

// Node-specific subscription callbacks. Concrete instances should define what actions these take,
// though we provide a very basic private member setter implementation in this baseclass
void NDNode::setRangeHandler(const num_sdk_base::NDRange::ConstPtr &msg)
{
	// Range-check inputs
	if (msg->min_range < 0.0f || 
		msg->max_range > 1.0f || 
		msg->min_range > msg->max_range)
	{
		ROS_ERROR("%s received invalid range settings (%.3f,%.3f)", getName().c_str(), msg->min_range, msg->max_range);
		return;
	}

	const bool updated_range = (msg->min_range != _min_range) || (msg->max_range != _max_range);

	if (true == updated_range)
	{
		_min_range = msg->min_range;
		_max_range = msg->max_range;
		ROS_DEBUG("%s updated range to (%.3f,%.3f)", getName().c_str(), msg->min_range, msg->max_range);
		publishStatus();
	}
}

void NDNode::setAngleHandler(const num_sdk_base::NDAngle::ConstPtr &msg)
{
	/* TODO: Figure out generic bounds checking for "angle"
	if (msg->angle_offset < 0.0f || 
		msg->total_angle < 0.0f  ||
		msg->angle_offset + msg->total_angle > 1.0)
	{
		ROS_ERROR("%s received invalid angle settings (%.3f,%.3f)", getName().c_str(), msg->angle_offset, msg->total_angle);
		return;
	}
	*/

	const bool updated_angle = (msg->angle_offset != _angle_offset) || (msg->total_angle != _total_angle);
	if (true == updated_angle)
	{
		_angle_offset = msg->angle_offset;
		_total_angle = msg->total_angle;
		ROS_DEBUG("%s updated angle to (%.3f,%.3f)", getName().c_str(), msg->angle_offset, msg->total_angle);
		publishStatus();
	}
}

static bool autoManualMsgIsValid(const num_sdk_base::NDAutoManualSelection::ConstPtr &msg)
{
	return (msg->adjustment >= 0.0f && msg->adjustment <= 1.0f);
}

void NDNode::setResolutionHandler(const num_sdk_base::NDAutoManualSelection::ConstPtr &msg)
{
	if (false == autoManualMsgIsValid(msg))
	{
		ROS_ERROR("%s received invalid resolution settings (adjustment = %.3f)", getName().c_str(), msg->adjustment);
		return;
	}

	const bool updated_resolution = (msg->enabled != _manual_resolution_enabled) || (msg->adjustment != _manual_resolution);
	if (true == updated_resolution)
	{
		_manual_resolution_enabled = msg->enabled;
		_manual_resolution = msg->adjustment;
		ROS_DEBUG("%s updated resolution to %s:%.3f", getName().c_str(), 
			(_manual_resolution_enabled)? "manual":"auto", 
			(_manual_resolution_enabled)? _manual_resolution : 0.0f);
		publishStatus();
	}
}

void NDNode::setGainHandler(const num_sdk_base::NDAutoManualSelection::ConstPtr &msg)
{
	if (false == autoManualMsgIsValid(msg))
	{
		ROS_ERROR("%s received invalid gain settings (adjustment = %.3f)", getName().c_str(), msg->adjustment);
		return;
	}

	const bool updated_gain = (msg->enabled != _gain_enabled) || (msg->adjustment != _gain);
	if (true == updated_gain)
	{
		_gain_enabled = msg->enabled;
		_gain = msg->adjustment;
		ROS_DEBUG("%s updated gain to %s:%.3f", getName().c_str(), 
			(_gain_enabled)? "enabled":"disabled", 
			(_gain_enabled)? _gain : 0.0f);
		publishStatus();
	}
}

void NDNode::setFilterHandler(const num_sdk_base::NDAutoManualSelection::ConstPtr &msg)
{
	if (false == autoManualMsgIsValid(msg))
	{
		ROS_ERROR("%s received invalid filter settings (adjustment = %.3f)", getName().c_str(), msg->adjustment);
		return;
	}

	const bool updated_filter = (msg->enabled != _filter_enabled) || (msg->adjustment != _filter_control);
	if (true == updated_filter)
	{
		_filter_enabled = msg->enabled;
		_filter_control = msg->adjustment;
		ROS_DEBUG("%s updated filter settings to %s:%.3f", getName().c_str(), 
			(_filter_enabled)? "enabled":"disabled", 
			(_filter_enabled)? _filter_control : 0.0f);
		publishStatus();
	}
}

void NDNode::publishImage(IMG_ID id, cv::Mat *img, ros::Time *tstamp, const std::string encoding)
{
	static uint32_t seq_ctr = 0;
	static const std::string frame_id = getName() + "_frame";
	
	// Build the header
	std_msgs::Header hdr;
	hdr.seq = seq_ctr;
	hdr.stamp = *tstamp;
	hdr.frame_id = frame_id;

	image_transport::Publisher *publisher = nullptr;
	cv::Mat *img_out = nullptr;
	// Choose an appropriate publisher and output image based on id and _simulated_data
	switch (id)
	{
	case IMG_0:
		publisher = &img_0_pub;
		img_out = (true == _simulated_data)? &img_0_sim_data : img;
		break;
	case IMG_1:
		publisher = &img_1_pub;
		img_out = (true == _simulated_data)? &img_1_sim_data : img;
		break;
	case IMG_ALT:
		publisher = &img_alt_pub;
		img_out = (true == _simulated_data)? &img_alt_sim_data : img;
		break;
	default:
		ROS_ERROR("%s: Request to publish unknown image id (%d)", getName().c_str(), id);
		return; // Don't increment sequence counter
	}

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(hdr, encoding, *img_out).toImageMsg();
	publisher->publish(msg);
	++seq_ctr;
}

void NDNode::publishStatus()
{
	num_sdk_base::NDStatus msg;

	msg.display_name = _display_name;
	
	msg.save_data_dir = _save_data_dir;
	
	msg.save_data.save_continuous = _save_continuous;
	msg.save_data.save_raw = _save_raw;

	msg.pause_enable = _paused;
	
	msg.simulate_data = _simulated_data;
	
	msg.range.min_range = _min_range;
	msg.range.max_range = _max_range;
	
	msg.angle.angle_offset = _angle_offset;
	msg.angle.total_angle = _total_angle;
	
	msg.resolution_settings.enabled = _manual_resolution_enabled;
	msg.resolution_settings.adjustment = _manual_resolution;
	
	msg.gain_settings.enabled = _gain_enabled;
	msg.gain_settings.adjustment = _gain;
	
	msg.filter_settings.enabled = _filter_enabled;
	msg.filter_settings.adjustment = _filter_control;

	_status_pub.publish(msg);
}

} // namespace numurus