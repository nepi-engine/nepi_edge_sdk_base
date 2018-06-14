#ifndef _SDK_NODE_H
#define _SDK_NODE_H

#include <string>

#include <ros/ros.h>
#include "std_msgs/Empty.h"

#include "num_common.h"

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
	 * These handles must have a lifetime as long as the NodeHandle, so are best approriated to a member variable container
	 */
	std::vector<ros::ServiceServer> servicers;

	/**
	 * Vector of return handles from NodeHandle::subscribe.
	 * These handles must have a lifetime as long as the NodeHandle, so are best approriated to a member variable container
	 */
	std::vector<ros::Subscriber> subscribers;

	/**
	 * @brief      Convenience function to properly namespace a parameter
	 *
	 * @param[in]  param  The parameter name (without namespace prefix)
	 *
	 * @return     The parameter name properly adorned with namespace prefix.
	 */
	inline const std::string getNodeParamName(const std::string &param) const
	{
		static const std::string ns_separator("/");
		return BASE_ROS_NAMESPACE + ns_separator + name + ns_separator + param;
	}

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
	 * Subclasses may override this method to do specific param initialization, but should call SDKNode::initParams() wihin the overriden implementation
	 * to ensure the generic initialization proceeds.
	 */
	virtual void initParams();

	/**
	 * @brief      Synchronize parameter values with the values in the param server.
	 * 
	 *			   This method must ensure that parameters and any underlying associated resources are in sync with values in the ROS param server.
	 *			   It is automatically installed as a callback to the reinit_params topic at both the global namespace level (for reiniting all nodes'
	 *			   params) and the node's private namespace level (for reiniting just this node's params)
	 *
	 * @param[in]  empty    Just a ROS-required placeholder
	 */
	void reinitParams(const std_msgs::Empty::ConstPtr& empty);
	
	/**
	 * @brief      Synchronize parameter server with the values in this node.
	 * 
	 *			   This method ensures that the ROS param server is updated with the current param values for this node.
	 *			   It is automatically installed as a callback to the update_params topic at both the global namespace level (for updating all nodes'
	 *			   params) and the node's private namespace level (for updating just this node's params)
	 *
	 * @param[in]  empty    Just a ROS-required placeholder
	 */
	virtual void updateParams(const std_msgs::Empty::ConstPtr& empty);

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

};

} // namespace Numurus

#endif //_SDK_NODE_H