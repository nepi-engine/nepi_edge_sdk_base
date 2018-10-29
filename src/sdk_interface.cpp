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
