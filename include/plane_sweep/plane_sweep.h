//
// Created by brage on 04.05.18.
//

#ifndef PLANE_SWEEP_PLANE_SWEEP_H
#define PLANE_SWEEP_PLANE_SWEEP_H

#include <opencv2/core.hpp>
#include "dataset.h"

struct DepthPlane
{
  double depth;
  double n_x;
  double n_y;
  double n_z;
};

struct DepthRange
{
  double min;
  double max;
  double interval;
};

class PlaneSweep
{
public:
  PlaneSweep();

  PlaneSweep(Dataset images);

  void addImage(DataElement image);


  cv::Mat getDepth(DepthPlane depth_plane);

  void setDepthNormal(double n_x, double n_y, double n_z);

private:

  cv::Mat getHomography(DepthPlane plane);

  cv::Mat mapTargetImgToRefImg(const cv::Mat& img, cv::Mat homography);

  int computeSimilarity(const cv::Mat& img_ref, const cv::Mat& img_target);

  cv::Mat computeFigureOfMerit(std::vector<int> similarity_measures);

  int findBestFitDepth();

  double ZNCC(const cv::Mat& img1, const cv::Mat& img2, const cv::Point2i& kernel_centre1, const cv::Point2i& kernel_centre2, int kernel_size);

};


#endif //PLANE_SWEEP_PLANE_SWEEP_H
