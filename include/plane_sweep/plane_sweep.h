//
// Created by brage on 04.05.18.
//

#ifndef PLANE_SWEEP_PLANE_SWEEP_H
#define PLANE_SWEEP_PLANE_SWEEP_H

#include <opencv2/core.hpp>
#include "dataset.h"

struct DepthRange
{
  double min;
  double max;
  double interval;
};

class PlaneSweep
{
public:
  PlaneSweep(Dataset images, int max_number_of_images = 3);

  PlaneSweep(std::vector<DataElement> images);

  void setDepthNormal(const Eigen::Vector3d& depth_normal);

  void setDepthRange(const DepthRange& depth_range);

  void setReferenceImage(int image_index);

  void setReferenceImage(const DataElement& ref_image);

  void calculateDepthImage();

private:
  Eigen::Vector3d depth_normal_; //n_m

  std::vector<DataElement> images_; //I_0, ..., I_k, ..., I_n

  DepthRange depth_range_; //m

  DataElement ref_image_;//I_ref

  int ZNCC_patch_size_;

  cv::Mat getHomography(Eigen::Vector3d depth_normal);

  cv::Mat mapTargetImgToRefImg(const cv::Mat& img, cv::Mat homography);

  cv::Mat mapTargetImgToRefImg(const DataElement& image, const DataElement& ref_image);

  int computeSimilarity(const cv::Mat& img_ref, const cv::Mat& img_target);

  cv::Mat computeFigureOfMerit(std::vector<int> similarity_measures);

  int findBestFitDepth();

  double ZNCC(const cv::Mat& img1, const cv::Mat& img2, const cv::Point2i& kernel_centre1, const cv::Point2i& kernel_centre2, int kernel_size);

};


#endif //PLANE_SWEEP_PLANE_SWEEP_H
