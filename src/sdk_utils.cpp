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
#include <math.h>

#include "sdk_utils.h"

namespace Numurus
{

// RunningStat class
RunningStat::RunningStat()
{};

void RunningStat::clear()
{
  count = 0; // Everything else gets cleared in push()
};

void RunningStat::push(double x)
{
  ++count;

  // See Knuth TAOCP vol 2, 3rd edition, page 232
  if (count== 1)
  {
      oldM = newM = maxVal = minVal = x;
      oldS = 0.0;
  }
  else
  {
      newM = oldM + (x - oldM)/count;
      newS = oldS + (x - oldM)*(x - newM);

      if (x > maxVal) maxVal = x;
      if (x < minVal) minVal = x;

      // set up for next iteration
      oldM = newM;
      oldS = newS;
  }
}

size_t RunningStat::getCount() const
{
  return count;
}

double RunningStat::mean() const
{
  return (count> 0) ? newM : 0.0;
}

double RunningStat::variance() const
{
  return ( (count> 1) ? newS/(count- 1) : 0.0 );
}

double RunningStat::stdDeviation() const
{
  return sqrt( variance() );
}

double RunningStat::max() const
{
  return maxVal;
}

double RunningStat::min() const
{
  return minVal;
}

} // namespace Numurus
