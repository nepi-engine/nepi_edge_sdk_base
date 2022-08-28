#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // Debugging only

#include "algorithms/underwater_img_enhancer.h"

#define IMG_CHANNEL_COUNT   4
namespace Numurus
{

UnderwaterImgEnhancer::UnderwaterImgEnhancer(size_t averaging_window_size) :
  alpha_averager{averaging_window_size},
  beta_averager{averaging_window_size}
{
  for (size_t i = 0; i < 3; ++i)
  {
    channel_offset_averager.emplace(channel_offset_averager.end(), averaging_window_size);
    channel_scale_averager.emplace(channel_scale_averager.end(), averaging_window_size);
  }
}

UnderwaterImgEnhancer::~UnderwaterImgEnhancer()
{}

bool UnderwaterImgEnhancer::applyImgAutoCorrect(cv::Mat &img_in_out, float sensitivity_ratio)
{
  // Sanity check
  const int input_channel_count = img_in_out.channels();
  if (input_channel_count != IMG_CHANNEL_COUNT)
  {
    printf("Invalid channel count (%d) for underwaterImgAutoCorrect\n", input_channel_count);
    return false;
  }

  cv::Mat channels[IMG_CHANNEL_COUNT];
  cv::split(img_in_out, channels);

  printf("!!! Debug: Split the image into channels !!!\n");

  double min_channel[IMG_CHANNEL_COUNT];
  double max_channel[IMG_CHANNEL_COUNT];
  double mean_channel[IMG_CHANNEL_COUNT];
  double minmax_channel = 255;

  // Gather the per-channel extrema and average
  for (size_t k = 0; k < 3; ++k) // Only do the color channels
  {
    cv::minMaxLoc(channels[k], &(min_channel[k]), &(max_channel[k]));
    mean_channel[k] = cv::mean(channels[k])[0];
    if (max_channel[k] < minmax_channel)
    {
      minmax_channel = max_channel[k];
    }
  }

  // Color Correction Optimization
  for (size_t k = 0; k < 3; ++k) // Only do the color channels
  {
    double chan_scale = (255.0 - mean_channel[k]) / (max_channel[k] - min_channel[k]);

    if (chan_scale < 1.0)
    {
      chan_scale = 1.0 - (1.0 - chan_scale) * (255.0 - minmax_channel) / 170.0;
    }
    else if (chan_scale > 1.0)
    {
      chan_scale = 1.0 + (chan_scale - 1.0) * (255.0 - minmax_channel) / 170.0;
    }

    if (chan_scale > (1 + sensitivity_ratio))
    {
      chan_scale = 1 + sensitivity_ratio;
    }
    else if (chan_scale < -(1+sensitivity_ratio))
    {
      chan_scale = -(1+sensitivity_ratio);
    }

    double chan_offset = -(min_channel[k]);
    //printf("Debug: chan_offset[%zu] before limits = %f\n", k, chan_offset);

    if (chan_offset < (-10 * (1 + 9*sensitivity_ratio)))
    {
      chan_offset = (int)(-10 * (1 + 9*sensitivity_ratio));
    }

    // Apply averaging
    chan_scale = channel_scale_averager[k].calculateNext(chan_scale);
    chan_offset = channel_offset_averager[k].calculateNext(chan_offset);
    //fprintf(stderr, "!!! Debug: chan_scale = %f, chan_offset = %f\n", chan_scale, chan_offset);
    cv::add(channels[k], chan_offset, channels[k]);
    channels[k] *= chan_scale;
  }

  //printf("!!! Debug: Rescaled the channels !!!\n");

  // Put the channels back together as a single image
  cv::merge(channels, IMG_CHANNEL_COUNT, img_in_out);
  img_in_out.convertTo(img_in_out, CV_8UC4); // Assumes BGRA
  //cv::imshow("Debug - Merged", img_in_out);

  // Calculate grayscale historgram
  cv::Mat gray_img;
  cv::cvtColor(img_in_out, gray_img, cv::COLOR_BGRA2GRAY);
  //cv::imshow("Debug - Gray", gray_img);

  const int hist_channels = 0;
  const int hist_size = 256;
  float hist_range[] = {0,256};
  const float *hist_ranges[] = {hist_range};
  cv::Mat hist;
  cv::calcHist(&gray_img, 1, &hist_channels, cv::Mat(), hist, 1, &hist_size, hist_ranges, true, false);
  //printf("!!! Debug: Got the histogram !!!\n");

  const size_t hist_length = hist.rows * hist.cols;

  // Debugging
  /*
  for (size_t i = 0; i < hist_length; ++i)
  {
    printf("Debug - %zu = %d\n", i, hist.at<int>(i));
  }
  */

  // Calculate cumulative distribution
  std::vector<float> accumulator;
  accumulator.push_back((float)hist.at<int>(0));
  for (size_t i = 1; i < hist_length; ++i)
  {
    accumulator.push_back(accumulator[i - 1] + (float)hist.at<int>(i));
  }

  // Locate cut points
  const float max_cumulative = accumulator[hist_length - 1];
  const float clip_hist_percent = max_cumulative / 200; // Magic number?
  int min_gray_index = 0;
  while (accumulator[min_gray_index] < clip_hist_percent)
  {
    min_gray_index += 5;
  }

  int max_gray_index = hist_length - 1;
  while (accumulator[max_gray_index] > (max_cumulative - clip_hist_percent))
  {
    max_gray_index -= 1;
  }

  // Calculate alpha and beta
  float alpha = 255.0 / (max_gray_index - min_gray_index) * (0.5+sensitivity_ratio);
  float beta = (-min_gray_index * alpha + 10) * (0.5+sensitivity_ratio);

  // Apply averaging
  alpha = alpha_averager.calculateNext(alpha);
  beta = beta_averager.calculateNext(beta);

  //fprintf(stderr, "!!! Debug: alpha = %f, beta = %f\n", alpha, beta);
  // Rescale
  cv::convertScaleAbs(img_in_out, img_in_out, alpha, beta);

  return true;
}

void UnderwaterImgEnhancer::updateAveragingWindowSize(size_t window_size)
{
  for (auto &averager : channel_offset_averager)
  {
    averager.updateWindowSize(window_size);
  }

  for (auto &averager : channel_scale_averager)
  {
    averager.updateWindowSize(window_size);
  }
  alpha_averager.updateWindowSize(window_size);
  beta_averager.updateWindowSize(window_size);
}

void UnderwaterImgEnhancer::resetAveraging()
{
  for (auto &averager : channel_offset_averager)
  {
    averager.reset();
  }

  for (auto &averager : channel_scale_averager)
  {
    averager.reset();
  }
  alpha_averager.reset();
  beta_averager.reset();
}

} // namespace numurus

#ifdef UNDERWATER_IMG_ENHANCER_TEST_APP
#include <opencv2/highgui.hpp>
#include <chrono>

void printUsage(char *prog_name)
{
  printf("Usage: %s <input_img_filename> <sensitivity_ratio>\n", prog_name);
}

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    printUsage(argv[0]);
    return -1;
  }

  cv::Mat img = cv::imread(argv[1]);
  if (img.empty())
  {
    printf("Error: Unable to open image %s\n", argv[1]);
    printUsage(argv[0]);
    return -1;
  }

  const float sensitivity_ratio = atof(argv[2]);
  if ((sensitivity_ratio < 0.0f) || (sensitivity_ratio > 1.0f))
  {
    printf("Error: Invalid sensitivity ratio (%.3f)... must be between 0.0 and 1.0\n", sensitivity_ratio);
    printUsage(argv[0]);
    return -1;
  }

  //cv::imshow("Before enhancement", img);
  const auto start = std::chrono::high_resolution_clock::now();
  Numurus::underwaterImgAutoCorrect(img, sensitivity_ratio))
  const auto stop = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> ms_double = stop - start;
  printf("Image enhancement took %.3fms\n", ms_double.count());

  cv::imshow("After enhancement", img);
  int k = cv::waitKey(0);

  return 0;
}

#endif
