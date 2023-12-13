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
#pragma once

#include <vector>
#include <opencv2/core/mat.hpp>

#include <algorithms/approx_moving_avg.h>

namespace Numurus
{

enum ImgEnhancerAlgorithm
{
  IMG_ENH_ALG_SHIFT_ONLY = 0,
  IMG_ENH_ALG_SHIFT_AND_SCALE = 1
};

class UnderwaterImgEnhancer
{
public:
  UnderwaterImgEnhancer(size_t averaging_window_size);
  ~UnderwaterImgEnhancer();

  bool applyImgAutoCorrect(cv::Mat &img_in_out, float sensitivity_ratio = 0.5f);
  void updateAveragingWindowSize(size_t window_size);
  void resetAveraging();
  bool setAlgorithm(ImgEnhancerAlgorithm alg);

private:
  std::vector<ApproxMovingAvg> channel_offset_averager;
  std::vector<ApproxMovingAvg> channel_scale_averager;
  ApproxMovingAvg alpha_averager;
  ApproxMovingAvg beta_averager;

  ImgEnhancerAlgorithm algorithm = IMG_ENH_ALG_SHIFT_ONLY;
};

} // namespace Numurus
