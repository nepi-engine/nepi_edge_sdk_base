/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
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
