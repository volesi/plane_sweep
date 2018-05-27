//
// Created by brage on 04.05.18.
//

#include <opencv2/imgproc.hpp>
#include "../include/plane_sweep/plane_sweep.h"

namespace a
{
double getAverage(const cv::Mat& img, const cv::Point2i& kernel_centre, int kernel_size);
double getStdDeviation(const cv::Mat& img, const cv::Point2i& kernel_centre, int kernel_size);
cv::Mat getStdDeviation(const cv::Mat& img, int kernel_size, cv::Mat avg = cv::Mat());
}

void PlaneSweep::calculateDepthImage()
{
  cv::Mat depth_image;

  for (const auto& image : images_)
  {
    if(image.img_num == ref_image_.img_num)
    { continue; }

    for (double d_m = depth_range_.min; d_m < depth_range_.max; d_m += depth_range_.interval)
    {


    }
    //for each depth in range{
      //calculate d_m

      //calculate homography for depth (ref_image, image)

      //warp image to ref image (image, homography)

      //calculate fitting score for each pixel in the warped image
      //each pixel in the ref image get a graph og heigths

    //}

  }

}

double PlaneSweep::ZNCC(const cv::Mat& img_ref, const cv::Mat& img_target, int kernel_size, cv::Mat avg_ref,
                        cv::Mat stddev_ref)
{
  if(avg_ref.empty())
  { cv::blur(img_ref, avg_ref, {kernel_size, kernel_size}); }

  if(stddev_ref.empty())
  {

  }

//  const auto &u1 = kernel_centre1.x;
//  const auto &v1 = kernel_centre1.y;
//  const auto &u2 = kernel_centre2.x;
//  const auto &v2 = kernel_centre2.y;

//  const auto std_dev1 = getStdDeviation(img1, kernel_centre1, kernel_size);
//  const auto std_dev2 = getStdDeviation(img2, kernel_centre2, kernel_size);
//  const auto avg1 = getAverage(img1, kernel_centre1, kernel_size);
//  const auto avg2 = getAverage(img2, kernel_centre2, kernel_size);

//  double sum = 0;
//  for (int i = -kernel_size; i < kernel_size + 1; ++i)
//  {
//    for (int j = -kernel_size; j < kernel_size + 1; ++j)
//    {
//      sum += (img_ref.at<char>(u1+i, v1+j) - avg1)*(img_target.at<char>(u2+i, v2+j) - avg2);
//    }
//  }
//
//  return sum / (std::pow(2*kernel_size + 1, 2) * std_dev1 * std_dev2);
  return 0.;
}

namespace a
{
double getAverage(const cv::Mat& img, const cv::Point2i& kernel_centre, int kernel_size)
{

  double sum = 0;
  const auto &u = kernel_centre.x;
  const auto &v = kernel_centre.y;

  for (int i = -kernel_size; i < kernel_size + 1; ++i)
  {
    for (int j = -kernel_size; j < kernel_size + 1; ++j)
    {
      sum += img.at<char>(u+i , v+j);
    }
  }
  return sum / std::pow(2*kernel_size + 1, 2);
}

cv::Mat getStdDeviation(const cv::Mat& img, int kernel_size, cv::Mat avg)
{
  assert(kernel_size % 2 == 1); //kernel_size must be odd

  if(avg.empty())
  { cv::blur(img, avg, {kernel_size, kernel_size}); }
  else
  { assert(img.size == avg.size); }

  cv::Mat variance;
  cv::blur(img.mul(img), variance, {kernel_size, kernel_size});

  cv::Mat stddev;
  cv::sqrt(variance - avg.mul(avg), stddev);

  return stddev;
}

double getStdDeviation(const cv::Mat& img, const cv::Point2i& kernel_centre, int kernel_size)
{
  double sum = 0;
  const auto &u = kernel_centre.x;
  const auto &v = kernel_centre.y;

  const auto avg = getAverage(img, kernel_centre, kernel_size);

  for (int i = -kernel_size; i < kernel_size + 1; ++i)
  {
    for (int j = -kernel_size; j < kernel_size + 1; ++j)
    {
      sum += std::pow(img.at<char>(u+i , v+j) - avg, 2);
    }
  }

  return std::pow(sum, 0.5) / (2*kernel_size + 1);
}
}
