#ifndef _SDK_NODE_H
#define _SDK_NODE_H

#include <string>

#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "num_sdk_base/Reset.h"

namespace Numurus
{
/**
 * @brief      Base class to represent a ROS node in the Numurus SDK
 * 
 * 			   This class encapsulates the components required for a ROS node, including both a public namespace and private namespace NodeHandle, and
 * 			   vectors of subscribers, servicers, etc. for ROS operations.
 * 			   
 * 			   Concrete base classes must provide implementations for the run() method and may provide implementations for the init() method.
 */
class SDKNode
{
public:
/**
 * @brief      Class to represent node parameters
 *
 *			   This class is useful primarily to provide a smart assignment operator that will respect the config_rt setting.
 *			   It also provides some utility in its encapsulation of the param name, helping to avoid a lot of strings peppered
 *			   through the code.
 * @tparam     T     Parameter type. Must be a valid ROS param type.
 */
template <class T>
class NodeParam
{
public:
	NodeParam(std::string param_name, T default_val, SDKNode *parent):
		_param_name{param_name},
		_param_data{default_val},
		_parent{parent}
	{}
	
	/**
	 * @brief      Conversion operator to allow a NodeParam to be implicitly converted to its template type
	 */
	operator T() {return _param_data;}

	/**
	 * @brief      Retrieves a parameter from the param server
	 * 
	 * 			   If param server has no value for requested param, this will establish it in the param server from value of param_storage.
	 *
	 * @return     true if parameter was found on server, false otherwise (but param thereafter exists on server)
	 */
	bool retrieve()
	{
		if (true == _parent->n_priv.getParam(_param_name, _param_data))
		{
			ROS_INFO("%s: Updating %s from param server", _parent->getName().c_str(), _param_name.c_str());
			return true;
		}
		else
		{
			ROS_WARN("%s: unable to init %s from param server, using existing val instead", _parent->getName().c_str(), _param_name.c_str());
			// And attempt write it back so that the config file has something for next time
			_parent->n_priv.setParam(_param_name, _param_data);
		}
		return false;
	}

	/**
	 * @brief      Assignment operator to allow assignment from an instance of the template type
	 *
	 * @param[in]  rhs   The right hand side
	 *
	 * @return     Reference to this NodeParam instance
	 */
	NodeParam& operator=(const T& rhs)
	{
		_param_data = rhs;
		_parent->n_priv.setParam(_param_name, _param_data);
		if (true == _parent->_save_cfg_rt)
		{
			std_msgs::String node_name;
			node_name.data = _parent->getName();
			_parent->_store_params_pub.publish(node_name);
		}
		return *this;
	}

private:
	const std::string _param_name;
	T _param_data;
	const SDKNode *_parent;
}; // class NodeParam

	/**
	 * @brief      Constructs the object.
	 *
	 */
	SDKNode();
	~SDKNode();

	/**
	 * @brief      Execute the main work method
	 * 
	 * 			   This base implementation simply calls init() and ros::spin(). 
	 * 			   
	 * @note       {Subclasses may override this methods but must ensure that they call init() and some version of ros::spin()}			   
	 */
	virtual void run();
	
	/**
	 * @brief      Get the name of the SDK Node.
	 *
	 * @return     The name.
	 */
	inline const std::string& getName() const {return ros::this_node::getName();}

	static std::vector<std::string> splitNamespace();

protected:
	/**
	 * The public namespace node handle. This is used for any ROS primitives that should resolve into the top-level namespace for this node. In particular, subscribing to
	 * general topics and services occurs through this node handle.
	 */
	ros::NodeHandle n;

	/**
	 * The private namespace node handle. This is used for any ROS primitives that must resolve into a subnamespace of this node to avoid conflicts. In particular, param access
	 * use this node handle.
	 */
	ros::NodeHandle n_priv; 

	/**
	 * Display name of the node. Could be modified by users (though no interface to do so for this generic base class)
	 */
	NodeParam<std::string> _display_name;

	/**
	 * Vector of return handles from NodeHandle::advertiseService.
	 * These handles must have a lifetime as long as the NodeHandle, so are best appropriated to a member variable container
	 */
	std::vector<ros::ServiceServer> servicers;

	/**
	 * Vector of return handles from NodeHandle::subscribe.
	 * These handles must have a lifetime as long as the NodeHandle, so are best appropriated to a member variable container
	 */
	std::vector<ros::Subscriber> subscribers;

	std::string device_type = "null_dev_type";
	std::string device_sn = "null_dev_sn";

	/**
	 * @brief      Advertise topics to be published
	 * 
	 * @note       {Subclasses may override this method, but should call back to the base
	 * 				class method to ensure the generic initialization proceeds.} 
	 */
	virtual void initPublishers();

	/**
	 * @brief      Retrieve all ROS parameters from param server.
	 * 
	 * 			   The ROS parameter server launches with parameters from the various config. files.
	 * 			   SDKNodes never interact with these files directly, rather gathering params from the
	 * 			   param server, and leveraging the config_mgr node to save and restore params to/from
	 * 			   the config files.
	 * 			   
	 * @note       {Subclasses may override this method to do specific param initialization, but should call SDKNode::retrieveParams() wihin the overriden implementation
	 * 				to ensure the generic initialization proceeds.}
	 */
	virtual void retrieveParams();

	/**
	 * @brief      Initialize subscribers and add to the list of subscriber handles
	 * 
	 * @note       {Subclasses may override this method, but should call back to the base
	 * 				class method to ensure the generic initialization proceeds.} 
	 */
	virtual void initSubscribers();

	/**
	 * @brief      Advertise services to be provided
	 * 
	 * @note       {Subclasses may override this method, but should call back to the base
	 * 				class method to ensure the generic initialization proceeds.} 
	 */
	virtual void initServices();

	/**
	 * @brief      Handle a request to save the current ROS configuration
	 *
	 * @param[in]  empty    Just a ROS-required placeholder
	 */
	void saveCfgHandler(const std_msgs::Empty::ConstPtr &empty);

	void saveCfgRtHandler(const std_msgs::Bool::ConstPtr &msg);
	
	/**
	 * @brief      Handle a request to reset the node
	 * 
	 * 			   The request can be one of a number of reset types. This methods simply calls into
	 * 			   reset type-specific functions that may be overridden by subclasses
	 *
	 * @param[in]  msg   The message containing the reset type requested
	 */
	void resetHandler(const num_sdk_base::Reset::ConstPtr &msg);

	/**
	 * @brief      Execute a user reset
	 * 			   
	 * 			   This generic implementation simply restores parameters to their param server values
	 * 			   which are nominally the same as the config file values that were loaded at start-up time.
	 * 			   
	 * 			   Subclasses do not typically need to override this method
	 */
	virtual void userReset();

	/**
	 * @brief      Exectue a factory reset, restoring configuration to it's factory default
	 * 
	 * 			   Subclasses do not typically need to override this method
	 */
	virtual void factoryReset();

	/**
	 * @brief      Execute a software reset, terminating the node and allowing the ROS launch system to restart it.
	 * 
	 * 			   Subclasses do not typically need  to override this method.
	 */
	virtual void softwareReset();

	/**
	 * @brief      Execute a hardware reset.
	 * 
	 * 			   This implementation of this reset mode is typically dependent on the underlying
	 * 			   hardware proxied by the node, so subclasses will typically override this method.
	 * 			   
	 * 			   The generic implementation included here simply calls softwareReset()
	 */		
	virtual void hardwareReset();

	/**
	 * @brief      Save the node's configuration file
	 * 
	 * 			   This method serves to coordinate a save operation with the parameter server by
	 * 			   informing the param server that it will be saving, updating all parameters (via updateParams), then 
	 * 			   informing the param server that it is done. This allows the param server to determine that a
	 * 			   set of nodes is attempting to save, then delay the actual file saving until they have finished updates.
	 */
	void saveCfg();


private:
	ros::Publisher _store_params_pub;
	bool _save_cfg_rt = false;
	
	/**
	 * @brief      Initialize the node
	 * 
	 * 			   This method calls initPublishers(), retrieveParams(), initSubscribers, and initServices() 
	 * 			   (in that order) to completely initialize the ROS components of the node. It is private, and
	 * 			   not called directly - instead the run() method calls this before spinning.
	 */	
	void init();
};

} // namespace Numurus

#endif //_SDK_NODE_H