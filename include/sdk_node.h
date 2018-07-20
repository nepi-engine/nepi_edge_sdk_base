#ifndef _SDK_NODE_H
#define _SDK_NODE_H

#include <string>

#include <ros/ros.h>
#include "std_msgs/Empty.h"

namespace Numurus
{

/**
 * @brief      Abstract base class to represent a ROS node in the Numurus SDK
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
	 * @brief      Constructs the object.
	 *
	 * @param[in]  name  The name of the node - namespace separation is handled automatically through launch files and class internals, so no namespace prefix should be provided
	 */
	SDKNode(const std::string name);
	~SDKNode();

	/**
	 * @brief      Initialize the node and execute the main work method, launching worker threads as needed
	 * 
	 * 			   This pure virtual method must be implemented by concrete subclasses to execute their particular tasks. Typically a run() method implementation consists of 
	 * 			   a call to init(), followed by registeration for various specialized ROS primitives (e.g., topics, services). Additional work threads that have a lifetime
	 * 			   as long as this node's should be launched as well. Finally, the run() method should call ros::spin() to kick off the event-driven system that executes callbacks 
	 * 			   as necessary
	 */
	virtual void run() = 0;
	
	/**
	 * @brief      Get the name of the SDK Node.
	 *
	 * @return     The name.
	 */
	inline const std::string& getName() const {return name;}

	/**
	 * @brief      Determines if initialized.
	 *
	 * @return     True if initialized, False otherwise.
	 */
	inline const bool isInitialized() const {return initialized;}

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
	 * Name of the node
	 */
	const std::string name;

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

	/**
	 * @brief      Initialize the SDK node
	 * 			   Typically this method is called from within the concrete run() implementation. This base class implementation does some
	 * 			   important work associated with initializing parameters, so any subclass override should call back to SDKNode::init() as it's
	 * 			   first call.
	 */		
	virtual void init();

	/**
	 * @brief      Initialize all parameters from config file values (if available).
	 * 
	 * @note       {Subclasses may override this method to do specific param initialization, but should call SDKNode::initParams() wihin the overriden implementation
	 * 				to ensure the generic initialization proceeds.}
	 */
	virtual void initParams();

	/**
	 * @brief      Synchronize parameter server with the values in this node.
	 * 
	 *			   This method ensures that the ROS param server is updated with the current param values for this node.
	 *			   It is automatically installed as a callback to the update_params topic at both the global namespace level (for updating all nodes'
	 *			   params) and the node's private namespace level (for updating just this node's params)
	 *			   
	 * @note       {Subclasses may override this method, but must ensure that they call back to this base class version.}			   
	 *
	 */
	virtual void updateParams();

	/**
	 * @brief      Handle a request to save the current ROS configuration
	 *
	 * @param[in]  empty    Just a ROS-required placeholder
	 */
	void saveCfgHandler(const std_msgs::Empty::ConstPtr &empty);

	/**
	 * @brief      Save the node's configuration file
	 * 
	 * 			   This method serves to coordinate a save operation with the parameter server by
	 * 			   informing the param server that it will be saving, updating all parameters (via updateParams), then 
	 * 			   informing the param server that it is done. This allows the param server to determine that a
	 * 			   set of nodes is attempting to save, then delay the actual file saving until they have finished updates.
	 */
	void saveCfg();

	/**
	 * @brief      Retrieves a parameter from the param server
	 * 
	 * 			   If param server has no value for requested param, this will set the default one from value of param_storaage.
	 *
	 * @param[in]  param_name     The parameter name
	 * @param[in,out] param_storage  Reference to storage for the param value - must be preinitialized to a reasonable value
	 *
	 * @tparam     T              Type - must be a valid overload type of ROS::NodeHandle::getParam()
	 */
	template <class T>
	void retrieveParam(const std::string param_name, T& param_storage)
	{
		if (true == n_priv.getParam(param_name, param_storage))
		{
			ROS_INFO("%s: Updating %s from config file", name.c_str(), param_name.c_str());
		}
		else
		{
			ROS_WARN("%s: unable to init %s from param server, using existing val instead", name.c_str(), param_name.c_str());
			// And attempt write it back so that the config file has something for next time
			n_priv.setParam(param_name, param_storage);
		}
	}

	/**
	 * @brief      Determines whether this node is fully initialized and ready for it's main task(s)
	 *
	 * @return     true if the node is ready, false otherwise.
	 */
	virtual bool ready();

private:
	/**
	 * true if initialization succeeded, false otherwise
	 */
	bool initialized;

	ros::Publisher _update_cfg_pending_pub;
	ros::Publisher _update_cfg_complete_pub;

};

} // namespace Numurus

#endif //_SDK_NODE_H