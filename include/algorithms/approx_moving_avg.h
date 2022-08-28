#pragma once

#include <chrono>

namespace Numurus
{

class ApproxMovingAvg
{
public:
  ApproxMovingAvg(size_t window_size);
  virtual ~ApproxMovingAvg();

  double calculateNext(double new_val);
  void updateWindowSize(size_t window_size);
  void reset();

protected:
  size_t val_count;
  double mean;
  double gamma;
};

}
