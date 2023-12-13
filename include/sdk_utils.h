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
#ifndef _SDK_UTILS_H
#define _SDK_UTILS_H

#define BOOL_TO_ENABLED(x)	((x)==true)? "enabled" : "disabled"
#define DEG_TO_RAD 0.01745329251f
#define RAD_TO_DEG 57.2958

namespace Numurus
{
// Following is lifted from John Cook's blog, in turn lifted from Knuth's TAOCP
class RunningStat
{
public:
  RunningStat();

  void clear();
  void push(double x);
  size_t getCount() const;
  double mean() const;
  double variance() const;
  double stdDeviation() const;
  double max() const;
  double min() const;

private:
  size_t count = 0;
  double oldM = 0.0;
  double newM = 0.0;
  double oldS = 0.0;
  double newS = 0.0;
  double maxVal = 0.0;
  double minVal = 0.0;
};

} // namespace Numurus

#endif
