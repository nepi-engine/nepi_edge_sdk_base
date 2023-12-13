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
#ifndef __LORD_AHRS_ROS_WRAPPER_H
#define __LORD_AHRS_ROS_WRAPPER_H

#include "sdk_node.h"
#include "lord_ahrs_driver.h"

namespace Numurus
{
class LORDAHRSRosWrapper : public SDKNode
{
public:
  LORDAHRSRosWrapper();
  virtual ~LORDAHRSRosWrapper();

  // SDK Node Overrides
  void init() override;
  void run() override;
  void retrieveParams() override;
  void initPublishers() override;

private:
  NodeParam<std::string> ahrs_serial_port;
  NodeParam<int> publish_rate_hz;
  NodeParam<float> ahrs_x_rot_offset_deg;
  NodeParam<float> ahrs_y_rot_offset_deg;
  NodeParam<float> ahrs_z_rot_offset_deg;

  ros::Publisher imu_pub;
  ros::Publisher odom_pub;
  ros::Publisher heading_pub;

  LORDAHRSDriver driver;

  void publishIMUandOdom();
};

} //namespace Numurus
#endif
