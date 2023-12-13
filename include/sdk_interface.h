/*
 * NEPI Dual-Use License
 * Project: nepi_edge_sdk_base
 *
 * This license applies to any user of NEPI Engine software
 *
 * Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
 * see https://github.com/numurus-nepi/nepi_edge_sdk_base
 *
 * This software is dual-licensed under the terms of either a NEPI software developer license
 * or a NEPI software commercial license.
 *
 * The terms of both the NEPI software developer and commercial licenses
 * can be found at: www.numurus.com/licensing-nepi-engine
 *
 * Redistributions in source code must retain this top-level comment block.
 * Plagiarizing this software to sidestep the license obligations is illegal.
 *
 * Contact Information:
 * ====================
 * - https://www.numurus.com/licensing-nepi-engine
 * - mailto:nepi@numurus.com
 *
 */
#ifndef _SDK_INTERFACE_H
#define _SDK_INTERFACE_H

#include <string>
#include <vector>

#include <ros/ros.h>

namespace Numurus
{
// Forward declaration
class SDKNode;

/**
 * @brief      Provides a base class for all ROS-based interfaces
 * 
 * 			   The general Numurus ROS architecture includes nodes and interfaces, where the interface includes all ROS IPC mechanisms. This
 * 			   base class provides the general API for subclass interfaces, which should be included by composition in relevant SDKNode subclasses
 */
class SDKInterface
{
public:
	/**
	 * @brief      Constructor
	 * 
	 * @note       Through a 'friend' declaration in SDKNode, instantiation through this constructor automatically
	 * 			   adds this interface instance into the set of interfaces associated with the parent SDKNode (which)
	 * 			   then initializes all interfaces by iterating over the set within its own init method.
	 *
	 * @param      parent          Pointer to the parent SDKNode 
	 * @param      parent_pub_nh   Pointer to the parent's public-namespaced node handle
	 * @param      parent_priv_nh  Pointer to the parent's private-namespaced node handle
	 */
	SDKInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh);
	
	SDKInterface() = delete; // No default constructor available

	virtual ~SDKInterface();

	/**
	 * @brief      Retrieves parameters from param server.
	 * 
	 * 			   This method is called automatically as part of the parent SDKNode's initialization.
	 * 			   Subclasses may override, but should always call back to their base class's implementation
	 * 			   as part of their own.
	 */
	virtual void retrieveParams(){};
	
	/**
	 * @brief      Advertises published topics
	 * 
	 * 			   This method is called automatically as part of the parent SDKNode's initialization.
	 * 			   Subclasses may override, but should always call back to their base class's implementation
	 * 			   as part of their own.
	 */
	virtual void initPublishers(){};

	/**
	 * @brief      Subscribes to topics and adds subscribers to servicers vector
	 * 
	 * 			   This method is called automatically as part of the parent SDKNode's initialization.
	 * 			   Subclasses may override, but should always call back to their base class's implementation
	 * 			   as part of their own.
	 */
	virtual void initSubscribers(){};
	
	/**
	 * @brief      Advertises services that this interface provides
	 * 
	 * 			   This method is called automatically as part of the parent SDKNode's initialization.
	 * 			   Subclasses may override, but should always call back to their base class's implementation
	 * 			   as part of their own.
	 */
	virtual void initServices(){};

	/**
	 * @brief      Attaches to services that this interface relies on
	 * 
	 * 			   This method is called automatically as part of the parent SDKNode's initialization.
	 * 			   Subclasses may override, but should always call back to their base class's implementation
	 * 			   as part of their own.
	 */
	virtual void initServiceClients(){};

protected:
	/**
	 * Pointer to the SDKNode that contains this interface instance
	 */
	SDKNode *_parent_node;

	/**
	 * Pointer to the parent SDKNode's public namespace node handle, for ROS interface components that
	 * are available at the public (i.e., device-level) namespace.
	 */
	ros::NodeHandle *_parent_pub_nh;

	/**
	 * Pointer to the parent SDKNode's private namespace node handle, for ROS interface components that
	 * are available at the private (i.e., node-level) namespace.
	 */
	ros::NodeHandle *_parent_priv_nh;

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

};
} // namespace Numurus

#endif