#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include "nd_node.h"
#include "trigger_interface.h"
#include "save_data_interface.h"

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
	_min_range{"min_range", 0.0f, this},
	_max_range{"max_range", 0.0f, this},
	_angle_offset{"angle_offset", 0.0f, this},
	_total_angle{"total_angle", 0.0f, this},
	_manual_resolution_enabled{"manual_resolution_enabled", true, this},
	_manual_resolution{"manual_resolution", 1.0f, this},
	_gain_enabled{"gain_enabled", true, this},
	_gain{"gain", 1.0f, this},
	_filter_enabled{"filter_enabled", true, this},
	_filter_control{"filter_control", 1.0f, this},
	_img_0_name{"img_0_name", "img_0", this},
	_img_1_name{"img_1_name", "img_1", this},
	_alt_img_name{"alt_img_name", "alt", this},
	_img_0_frame_id{"img_0_frame_id", "img_0_frame", this},
	_img_1_frame_id{"img_1_frame_id", "img_1_frame", this},
	_alt_img_frame_id{"alt_img_frame_id", "alt_img_frame", this},
	_img_width{"img_width", 1920, this},
	_img_height{"img_height", 1080, this},
	_img_encoding{"img_encoding", sensor_msgs::image_encodings::RGB8, this}
{
	_save_data_if = new SaveDataInterface(this, &n, &n_priv);

	// Load the sim mode files
	const std::string SIM_IMG_BASENAME = "/opt/numurus/ros/etc/" + getName() + "/sim_img_";
	
	const std::string IMG_0_SIM_FILENAME = SIM_IMG_BASENAME + "0.jpg";
	loadSimData(IMG_0_SIM_FILENAME, &img_0_sim_data);

	const std::string IMG_1_SIM_FILENAME = SIM_IMG_BASENAME + "1.jpg";
	loadSimData(IMG_1_SIM_FILENAME, &img_1_sim_data);

	const std::string IMG_ALT_SIM_FILENAME = SIM_IMG_BASENAME + "alt.jpg";
	loadSimData(IMG_ALT_SIM_FILENAME, &img_alt_sim_data);
}

NDNode::~NDNode()
{
	delete _save_data_if;
	_save_data_if = nullptr;
}

void NDNode::initPublishers()
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

	// Advertise the nd_status topic, using the overload form that provides a callback on new subscriber connection.
	// Want to always send a status update whenever a subscriber connects.
	_status_pub = n_priv.advertise<num_sdk_msgs::NDStatus>("nd_status", 3, boost::bind(&NDNode::publishStatus, this));
}

void NDNode::retrieveParams()
{
	// Call the base method
	SDKNode::retrieveParams(); 
	
	// Grab the nd_node parameters
	_simulated_data.retrieve();
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
	_img_0_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again
	_img_1_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again
	_alt_img_name.retrieve(); // already retrieved in initPublishers, but no harm in doing it again
	_img_0_frame_id.retrieve();
	_img_1_frame_id.retrieve();
	_alt_img_frame_id.retrieve();
	_img_width.retrieve();
	_img_height.retrieve();
	_img_encoding.retrieve();

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

void NDNode::initSubscribers()
{
	// Call the base method
	SDKNode::initSubscribers();

	// Now subscribe to the set of global nd controls
	// These versions are in the public namespace so that we can support global commands
	subscribers.push_back(n.subscribe("pause_enable", 3, &NDNode::pauseEnableHandler, this));
	subscribers.push_back(n.subscribe("simulate_data", 3, &NDNode::simulateDataHandler, this));

	// Now subscribe to the private namespace versions
	subscribers.push_back(n_priv.subscribe("pause_enable", 3, &NDNode::pauseEnableHandler, this));
	subscribers.push_back(n_priv.subscribe("simulate_data", 3, &NDNode::simulateDataHandler, this));

	// Also in the private namespace are the various generic "tweaks"
	subscribers.push_back(n_priv.subscribe("set_range", 3, &NDNode::setRangeHandler, this));
	subscribers.push_back(n_priv.subscribe("set_angle", 3, &NDNode::setAngleHandler, this));
	subscribers.push_back(n_priv.subscribe("set_resolution", 3, &NDNode::setResolutionHandler, this));
	subscribers.push_back(n_priv.subscribe("set_gain", 3, &NDNode::setGainHandler, this));
	subscribers.push_back(n_priv.subscribe("set_filter", 3, &NDNode::setFilterHandler, this));

	// And init the interface subscribers
	if (nullptr != _save_data_if)
	{
		_save_data_if->initSubscribers();
	}
	if (nullptr != _trig_if)
	{
		_trig_if->initSubscribers();
	}
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
void NDNode::setRangeHandler(const num_sdk_msgs::NDRange::ConstPtr &msg)
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

void NDNode::setAngleHandler(const num_sdk_msgs::NDAngle::ConstPtr &msg)
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

static bool autoManualMsgIsValid(const num_sdk_msgs::NDAutoManualSelection::ConstPtr &msg)
{
	return (msg->adjustment >= 0.0f && msg->adjustment <= 1.0f);
}

void NDNode::setResolutionHandler(const num_sdk_msgs::NDAutoManualSelection::ConstPtr &msg)
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

void NDNode::setGainHandler(const num_sdk_msgs::NDAutoManualSelection::ConstPtr &msg)
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

void NDNode::setFilterHandler(const num_sdk_msgs::NDAutoManualSelection::ConstPtr &msg)
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

void NDNode::publishImage(int id, cv::Mat *img, sensor_msgs::CameraInfoPtr cinfo, ros::Time *tstamp)
{
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(cinfo->header, _img_encoding, *img).toImageMsg();

	publishImage(id, msg, cinfo);
}

void NDNode::publishImage(int id, sensor_msgs::ImagePtr img, sensor_msgs::CameraInfoPtr cinfo)
{
	// TODO: This method is not threadsafe, but called by multiple threads in e.g., gs_multicam
	image_transport::CameraPublisher *publisher = nullptr;
	sensor_msgs::ImagePtr img_out = nullptr;
	switch (id)
	{
	case IMG_0:
		publisher = &img_0_pub;
		img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, _img_encoding, img_0_sim_data).toImageMsg() : img;
		break;
	case IMG_1:
		publisher = &img_1_pub;
		img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, _img_encoding, img_1_sim_data).toImageMsg() : img;
		break;
	case IMG_ALT:
		publisher = &img_alt_pub;
		img_out = (true == _simulated_data)? cv_bridge::CvImage(cinfo->header, _img_encoding, img_alt_sim_data).toImageMsg() : img;
		break;
	default:
		ROS_ERROR("%s: Request to publish unknown image id (%d)", getName().c_str(), id);
		return;
	}

	publisher->publish(img_out, cinfo);

	saveDataIfNecessary(id, img_out);
}

void NDNode::saveDataIfNecessary(int img_id, sensor_msgs::ImagePtr img)
{
	if (false == _save_data_if->saveContinuousEnabled())
	{
		return; // Nothing else to do
	}

	std::string image_identifier;
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
		ROS_ERROR("%s: Request to save for unknown image id (%d)", getName().c_str(), img_id);
	}

	cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(img, img->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
       
    // Capture the timestamp in a good format for filenames        
    const std::string display_name = _display_name;
    boost::posix_time::ptime posix_time = img->header.stamp.toBoost();
	std::string time_str = boost::posix_time::to_iso_extended_string(posix_time);
    
    // Create the filename - defer the extension so that we can adjust as necessary for save_raw
	std::stringstream qualified_filename_no_extension;
    qualified_filename_no_extension << _save_data_if->_save_data_dir << "/" << display_name << _save_data_if->getFilenamePrefix() << "_" << image_identifier << "_" << time_str;  
    
    bool success = cv::imwrite( qualified_filename_no_extension.str() + ".jpg",  cv_ptr->image ); // OpenCV uses extensions intelligently
	if (false == success)
	{
		ROS_ERROR_STREAM_THROTTLE(1, "Could not save " << qualified_filename_no_extension.str() << ".jpg"); 
	}
	
    // Save the lossless PNG if raw data saving enabled
    if (true == _save_data_if->saveRawEnabled())
    {
    	success = cv::imwrite( qualified_filename_no_extension.str() + ".png",  cv_ptr->image ); // OpenCV uses extensions intelligently
    	if (false == success)
		{
			ROS_ERROR_STREAM_THROTTLE(1, "Could not save " << qualified_filename_no_extension.str() << ".png"); 
		}    	
    }
}

void NDNode::publishStatus()
{
	num_sdk_msgs::NDStatus msg;

	msg.display_name = _display_name;
	
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