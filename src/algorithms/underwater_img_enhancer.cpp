#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // Debugging only

#include "algorithms/underwater_img_enhancer.h"

#define IMG_CHANNEL_COUNT   4
namespace Numurus
{

bool underwaterImgAutoCorrect(cv::Mat &img_in_out, float sensitivity_ratio)
{
  // Sanity check
  const int input_channel_count = img_in_out.channels();
  if (input_channel_count != IMG_CHANNEL_COUNT)
  {
    printf("Invalid channel count (%d) for underwaterImgAutoCorrect\n");
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

    cv::add(channels[k], chan_offset, channels[k]);
    channels[k] *= chan_scale;
  }

  printf("!!! Debug: Rescaled the channels !!!\n");

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
  printf("!!! Debug: Got the histogram !!!\n");

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
  const float alpha = 255.0 / (max_gray_index - min_gray_index) * (0.5+sensitivity_ratio);
  const float beta = (-min_gray_index * alpha + 10) * (0.5+sensitivity_ratio);

  // Rescale
  cv::convertScaleAbs(img_in_out, img_in_out, alpha, beta);

  return true;
}

/* Original python
# encoding=utf-8
import tkinter as tk
from tkinter import filedialog
import os
import natsort
import datetime

import numpy as np
import cv2


def image_auto_correct(img,sensitivity_ratio=0.5):

    # Color Correction optimization
    Max=[0,0,0]
    for k in range(0, 3):
        Max  = np.max(img[:,:,k])
    Min_Max_channel  = np.min(Max)
    for k in range(0, 3):
        Max_channel  = np.max(img[:,:,k])
        Min_channel  = np.min(img[:,:,k])
        Mean_channel = np.mean(img[:,:,k])
        Chan_scale = (255 - Mean_channel) / (Max_channel - Min_channel)

        if Chan_scale < 1:
            Chan_scale = 1 - (1-Chan_scale)*(255-Min_Max_channel)/170
        elif Chan_scale > 1:
            Chan_scale = 1 + (Chan_scale-1)*(255-Min_Max_channel)/170
        if Chan_scale > 1*(1+sensitivity_ratio):
            Chan_scale = 1 *(1+sensitivity_ratio)
        if Chan_scale < -1*(1+sensitivity_ratio):
            Chan_scale = -1 *(1+sensitivity_ratio)
        print(Chan_scale)
        Chan_offset = -1*Min_channel
        if Chan_offset < -10 * (1+9*sensitivity_ratio):
            Chan_offset = -10 * (1+9*sensitivity_ratio)

        img[:,:,k] = (img[:,:,k] + Chan_offset) * Chan_scale

    # Contrast and Brightness optimization
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Calculate grayscale histogram
    hist = cv2.calcHist([gray],[0],None,[256],[0,256])
    hist_size = len(hist)

    # Calculate cumulative distribution from the histogram
    accumulator = []
    accumulator.append(float(hist[0]))
    for index in range(1, hist_size):
        accumulator.append(accumulator[index -1] + float(hist[index]))

    # Locate points to clip
    maximum = accumulator[-1]
    clip_hist_percent = (maximum/100.0)
    clip_hist_percent /= 2.0

    # Locate left cut
    minimum_gray = 0
    while accumulator[minimum_gray] < clip_hist_percent:
        minimum_gray += 5

    # Locate right cut
    maximum_gray = hist_size -1
    while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
        maximum_gray -= 1

    # Calculate alpha and beta values
    alpha = 255 / (maximum_gray - minimum_gray) * (0.5+sensitivity_ratio)
    beta = (-minimum_gray * alpha + 10) * (0.5+sensitivity_ratio)

    img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
    return (img)


if __name__ == '__main__':

    starttime = datetime.datetime.now()


    # Get file path
    start_folder='D:\Data\Image_Enhance\Image Sets\TestDIU'

##    root = tk.Tk()
##    root.withdraw()
##    pathname = filedialog.askdirectory(initialdir=start_folder)
##    #print('Folder Name')
##    #print( pathname)
    pathname=start_folder


    path_in = pathname
    path_out = path_in + "/Enhanced4b"
    if os.path.isdir(path_out)==False:
        os.mkdir(path_out)
    img_files=[]
    label_files=[]
    for file in os.listdir(path_in):
        if file.endswith(tuple(['.png','.PNG','.jpg','.JPG'])) and file.find('depth')==-1: # excludes depth map files
              img_files.append(file)
        if file.endswith(tuple(['.txt'])): # excludes depth map files
              label_files.append(file)
    img_files =  natsort.natsorted(img_files)
    #label_files =  natsort.natsorted(label_files)

    for i in range(len(img_files)):
        file = img_files[i]
        #print('********    file   ********',file)
        img = cv2.imread(path_in +'/' + file)
        img_filtered=image_auto_correct(img,0)
        file_out=path_out + "/" + file[:-4] + '_Enhanced_0.png'
        #print(file_out)
        cv2.imwrite(file_out, img_filtered)

    for i in range(len(img_files)):
        file = img_files[i]
        #print('********    file   ********',file)
        img = cv2.imread(path_in +'/' + file)
        img_filtered=image_auto_correct(img,0.5)
        file_out=path_out + "/" + file[:-4] + '_Enhanced_1.png'
        #print(file_out)
        cv2.imwrite(file_out, img_filtered)

    for i in range(len(img_files)):
        file = img_files[i]
        #print('********    file   ********',file)
        img = cv2.imread(path_in +'/' + file)
        img_filtered=image_auto_correct(img,1)
        file_out=path_out + "/" + file[:-4] + '_Enhanced_2.png'
        #print(file_out)
        cv2.imwrite(file_out, img_filtered)

    Endtime = datetime.datetime.now()
    Time = Endtime - starttime
    print('Time', Time)
*/

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
