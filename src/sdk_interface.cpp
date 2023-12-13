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
#include "sdk_interface.h"
#include "sdk_node.h"

namespace Numurus
{

SDKInterface::SDKInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh):
	_parent_node{parent},
	_parent_pub_nh{parent_pub_nh},
	_parent_priv_nh{parent_priv_nh}
{ 
	// SDKNode grants friend permission to this constructor so that we can push ourselves onto its collection of SDKInterfaces
	_parent_node->sdk_interfaces.push_back(this);
}

SDKInterface::~SDKInterface(){}
}
