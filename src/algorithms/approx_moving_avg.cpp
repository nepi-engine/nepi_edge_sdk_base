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

#include <algorithms/approx_moving_avg.h>

#define TIME_AVG_WINDOW_SIZE    5

namespace Numurus
{

ApproxMovingAvg::ApproxMovingAvg(size_t window_size) :
  val_count{0},
  mean{0.0}
{
  updateWindowSize(window_size);
}

ApproxMovingAvg::~ApproxMovingAvg()
{}

double ApproxMovingAvg::calculateNext(double new_val)
{
  if (gamma <= 0.0) return new_val; // Means window_size was <= 1

  ++val_count;

  mean = gamma*mean + (1.0 - gamma)*new_val;
  const double bias_corrected_mean = mean / (1.0 - pow(gamma, val_count));

  return bias_corrected_mean;
}

void ApproxMovingAvg::updateWindowSize(size_t window_size)
{
  if (window_size < 1) window_size = 1; // Don't let gamma go negative
  gamma = static_cast<double>(window_size - 1) / window_size;
}

void ApproxMovingAvg::reset()
{
  val_count = 0;
  mean = 0;
}

} // namespace Numurus
