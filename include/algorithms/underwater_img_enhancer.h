#pragma once

#include <array>
#include <opencv2/core/mat.hpp>

namespace Numurus
{

bool underwaterImgAutoCorrect(cv::Mat &img_in_out, float sensitivity_ratio = 0.5f);

} // namespace Numurus
