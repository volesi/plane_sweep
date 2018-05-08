//
// Created by brage on 04.05.18.
//

#include "../include/plane_sweep/plane_sweep.h"

namespace
{
double getAverage(const cv::Mat& img, const cv::Point2i& kernel_centre, int kernel_size);
double getStdDeviation(const cv::Mat& img, const cv::Point2i& kernel_centre, int kernel_size);
}


double PlaneSweep::ZNCC(const cv::Mat& img1, const cv::Mat& img2, const cv::Point2i& kernel_centre1,
                        const cv::Point2i& kernel_centre2, int kernel_size)
{
  const auto &u1 = kernel_centre1.x;
  const auto &v1 = kernel_centre1.y;
  const auto &u2 = kernel_centre2.x;
  const auto &v2 = kernel_centre2.y;

  const auto std_dev1 = getStdDeviation(img1, kernel_centre1, kernel_size);
  const auto std_dev2 = getStdDeviation(img2, kernel_centre2, kernel_size);
  const auto avg1 = getAverage(img1, kernel_centre1, kernel_size);
  const auto avg2 = getAverage(img2, kernel_centre2, kernel_size);

  double sum = 0;
  for (int i = -kernel_size; i < kernel_size + 1; ++i)
  {
    for (int j = -kernel_size; j < kernel_size + 1; ++j)
    {
      sum += (img1.at<char>(u1+i, v1+j) - avg1)*(img2.at<char>(u2+i, v2+j) - avg2);
    }
  }

  return sum / (std::pow(2*kernel_size + 1, 2) * std_dev1 * std_dev2);
}

void PlaneSweep::calculateDepthImage()
{
  cv::Mat depth_image;

  for (const auto& image : images_)
  {
    if(image.img_num == ref_image_.img_num)
    { continue; }
    //for each depth in range{
      //calculate d_m

      //calculate homography for depth (ref_image, image, n_m, d_m)

      //warp image to ref image (image, homography)

      //calculate fitting score for each pixel in the warped image

    //}

  }

}

namespace
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
