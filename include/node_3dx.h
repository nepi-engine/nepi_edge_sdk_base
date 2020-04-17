#ifndef _NODE_3DX_H
#define _NODE_3DX_H

#include <string>

#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "image_transport/image_transport.h"
#include "opencv2/core/mat.hpp"

#include "sdk_node.h"

#include "num_sdk_msgs/SaveData.h"
#include "num_sdk_msgs/Range3DX.h"
#include "num_sdk_msgs/Angle3DX.h"
#include "num_sdk_msgs/AutoManualSelection3DX.h"
#include "num_sdk_msgs/Status3DX.h"

namespace Numurus
{
// Forward declarations
class TriggerInterface;
class SaveDataInterface;

/**
 * @brief      The Node3DX class represents a Numurus sensor capable of producing multidimensional imagery
 * 
 * 			   All Node3DXs adhere to a common ROS interface in the form of guaranteed published and 
 * 			   subscribed topics. This class provides a base from which sensor-specific implementation can
 * 			   be derived.
 * 			   
 * @note:	   This class inherits from abstract SDKNode, and does not implement all of the pure virtual 
 * 			   methods so, it is, itself, abstract.
 */
class Node3DX : public SDKNode
{
public:
	/**
	 * @brief      Constructor
	 *
	 */
	Node3DX();

	virtual ~Node3DX();

	enum IMG_ID
	{
		IMG_0,
		IMG_1,
		IMG_ALT
	};

	// TODO: Probably belongs in a utility class
	static void loadSimData(std::string filename, cv::Mat *out_mat);

protected:
	static constexpr auto SENSOR_TYPE_INDEX = 4;
	TriggerInterface *_trig_if = nullptr;
	SaveDataInterface *_save_data_if = nullptr;
	image_transport::ImageTransport img_trans;
	
	image_transport::CameraPublisher img_0_pub;
	image_transport::CameraPublisher img_1_pub;
	image_transport::CameraPublisher img_alt_pub;

	cv::Mat img_0_sim_data;
	cv::Mat img_1_sim_data;
	cv::Mat img_alt_sim_data;

	std::string ros_cam_color_encoding_name;

	SDKNode::NodeParam<bool> _simulated_data;

	SDKNode::NodeParam<float> _min_range;
	SDKNode::NodeParam<float> _max_range;

	SDKNode::NodeParam<float> _angle_offset;
	SDKNode::NodeParam<float> _total_angle;

	SDKNode::NodeParam<bool> _manual_resolution_enabled;
	SDKNode::NodeParam<float> _manual_resolution;

	SDKNode::NodeParam<bool> _gain_enabled;
	SDKNode::NodeParam<float> _gain;

	SDKNode::NodeParam<bool> _filter_enabled;
	SDKNode::NodeParam<float> _filter_control;

	SDKNode::NodeParam<std::string> _img_0_name;
	SDKNode::NodeParam<std::string> _img_1_name;
	SDKNode::NodeParam<std::string> _alt_img_name;

	SDKNode::NodeParam<std::string> _img_0_frame_id;
	SDKNode::NodeParam<std::string> _img_1_frame_id;
	SDKNode::NodeParam<std::string> _alt_img_frame_id;

	// Inherited from SDKNode
	virtual inline bool validateNamespace() override {return ns_tokens.size() > 4;}
	virtual void initPublishers() override;
	virtual void retrieveParams() override;
	virtual void initSubscribers() override;
	
	// Generic subscription callbacks. In many cases, the default implementation in this base class will
	// be sufficient, but subclasses can override as necessary (ensuring that they call back to the base
	// class or embed the base class logic as necessary).
	virtual void pauseEnableHandler(const std_msgs::Bool::ConstPtr &msg);
	void simulateDataHandler(const std_msgs::Bool::ConstPtr &msg);

	static inline bool autoManualMsgIsValid(const num_sdk_msgs::AutoManualSelection3DX::ConstPtr &msg)
	{
		return (msg->adjustment >= 0.0f && msg->adjustment <= 1.0f);
	}

	// Node-specific subscription callbacks. Concrete instances should define what actions these take,
	// though we provide a very basic private member setter implementation in this baseclass
	virtual void setRangeHandler(const num_sdk_msgs::Range3DX::ConstPtr &msg);
	virtual void setAngleHandler(const num_sdk_msgs::Angle3DX::ConstPtr &msg);
	virtual void setResolutionHandler(const num_sdk_msgs::AutoManualSelection3DX::ConstPtr &msg);
	virtual void setGainHandler(const num_sdk_msgs::AutoManualSelection3DX::ConstPtr &msg);
	virtual void setFilterHandler(const num_sdk_msgs::AutoManualSelection3DX::ConstPtr &msg);

	void publishStatus();

	void publishImage(int img_id, cv::Mat *img, sensor_msgs::CameraInfoPtr cinfo, ros::Time *tstamp, bool save_if_necessary = true);
	void publishImage(int img_id, sensor_msgs::ImagePtr img, sensor_msgs::CameraInfoPtr cinfo, bool save_if_necessary = true);

	virtual void saveDataIfNecessary(int img_id, sensor_msgs::ImagePtr img);

private:
	bool _paused = false;
	ros::Publisher _status_pub;
};

}

#endif
