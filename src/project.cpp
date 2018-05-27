#include "../include/plane_sweep/project.h"

#include "../include/plane_sweep/dataset.h"
#include "../include/plane_sweep/viewer_3d.h"
#include "../include/plane_sweep/plane_sweep.h"
#include "../include/plane_sweep/homography_estimator.h"
#include "../include/plane_sweep/feature_utils.h"

#include <opencv/cv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core/eigen.hpp"

struct ps_image
{
  ps_image(DataElement image)
      : image{image.image}
      , intrinsics{image.intrinsics}
  {
    cv::Mat Kcv, dist, undistorted;
    const Eigen::Matrix3d K = intrinsics.toCalibrationMatrix();
    cv::eigen2cv(K, Kcv);
    const auto D = intrinsics.toDistortionCoefficientVector();
    cv::eigen2cv(D, dist);
    cv::undistort(this->image, undistorted, Kcv, dist);
    this->image = undistorted;

    cv::Mat ref_gray;
    cv::cvtColor(image.image, ref_gray, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SURF::create();
    cv::Ptr<cv::Feature2D> desc_extractor = cv::xfeatures2d::SURF::create();

    detector->detect(ref_gray, keypoints);
    cv::KeyPointsFilter::retainBest(keypoints, 500);
    desc_extractor->compute(ref_gray, keypoints, descriptors);
  }

  cv::Mat image;
  Intrinsics intrinsics;

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};

cv::Mat getPlainSweepHomography(const ps_image& ref, const ps_image& target, const cv::Vec3d& n, double d_m);

cv::Mat mat2gray(const cv::Mat& src);

cv::Mat getStdDeviation(const cv::Mat& img, int kernel_size, cv::Mat avg = cv::Mat());

cv::Mat zncc(const cv::Mat& image_ref, const cv::Mat& image_target, int kernel_size);

cv::Matx33f getHomography(ps_image ref, ps_image target);

void getRotationAndTranslation(const ps_image& ref, const ps_image& target, cv::Mat& R, cv::Mat& t);

void extractMatchingPointsCv(const std::vector<cv::KeyPoint>& keypts1, const std::vector<cv::KeyPoint>& keypts2,
                             const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& matched_pts1,
                             std::vector<cv::Point2f>& matched_pts2);

Project::Project(const std::string& data_path)
    : data_path_{data_path}
    , window_name_{"World point in camera"}
{}

void Project::run()
{
  Dataset dataset{data_path_, {510, 512, 514}};
  cv::namedWindow("mosaic");

  DataElement right{};
  DataElement ref{};
  DataElement left{};

  dataset >> left;
  dataset >> ref;
  dataset >> right;

  //Set up reference image
  auto ps_ref = ps_image(ref);
  auto ps_left = ps_image(left);
  auto ps_right = ps_image(right);

  //Find homography got images
  cv::Mat H_m_cv_l = getPlainSweepHomography(ps_ref, ps_left, {0, 0, -1}, 1026);
  cv::Mat H_m_cv_r = getPlainSweepHomography(ps_ref, ps_right, {0, 0, -1}, 1026);

  //Warp targets to reference image
  cv::Mat ref_gray;
  cv::cvtColor(ps_ref.image, ref_gray, CV_BGR2GRAY);

  cv::Mat frame_warp_left, left_gray;
  cv::cvtColor(ps_left.image, left_gray, CV_BGR2GRAY);
  cv::warpPerspective(left_gray, frame_warp_left, H_m_cv_l, ref_gray.size());

  cv::Mat frame_warp_right, right_gray;
  cv::cvtColor(ps_right.image, right_gray, CV_BGR2GRAY);
  cv::warpPerspective(right_gray, frame_warp_right, H_m_cv_r, ref_gray.size());

  //Get proper colours on image
  std::vector<cv::Mat> images(3);
  images.at(0) = frame_warp_right; //for blue channel
  images.at(1) = ref_gray;   //for green channel
  images.at(2) = frame_warp_left;  //for red channel

  //Combine images
  cv::Mat colorImage;
  cv::merge(images, colorImage);

  // Show mosaicking result.
  cv::imshow("mosaic", colorImage);
  cv::waitKey();

  return;
}

cv::Mat zncc(const cv::Mat& image_ref, const cv::Mat& image_target, int kernel_size)
{
  assert(image_ref.type() == CV_32FC1);
  assert(image_target.type() == CV_32FC1);
  assert(kernel_size % 2 == 1);

  cv::Mat avg_ref;
  cv::blur(image_ref, avg_ref, {kernel_size, kernel_size});
  const auto stddev_ref = getStdDeviation(image_ref, kernel_size, avg_ref);

  cv::Mat avg_target;
  cv::blur(image_target, avg_target, {kernel_size, kernel_size});
  const auto stddev_target = getStdDeviation(image_target, kernel_size, avg_target);

  cv::Mat return_mat(image_ref.size(), image_ref.type(), {0.f, 0.f, 0.f});

  const int n = (kernel_size - 1) / 2;
  for (int v = n; v < return_mat.rows - n; ++v)
  {
    for (int u = n; u < return_mat.cols - n; ++u)
    {
      //for each pixel
      float sum = 0;
      const auto avg_ref_px = avg_ref.at<float>(v, u);
      const auto avg_target_px = avg_target.at<float>(v, u);

      for (int i = -n; i < n; ++i)
      {
        for (int j = -n; j < n; ++j)
        {
          sum += (image_ref.at<float>(v + i, u + j) - avg_ref_px) *
                 (image_target.at<float>(v + i, u + j) - avg_target_px);
        }
      }
      return_mat.at<float>(v, u) = sum;
    }
  }
  return_mat /= (std::pow(kernel_size, 2) * stddev_ref.mul(stddev_target));

  return return_mat;
}

cv::Mat getStdDeviation(const cv::Mat& img, int kernel_size, cv::Mat avg)
{
  assert(img.type() == CV_32FC1);
  assert(kernel_size % 2 == 1);

  if (avg.empty())
  { cv::blur(img, avg, {kernel_size, kernel_size}); }
  else
  { assert(img.size() == avg.size()); }

  cv::Mat variance;
  cv::blur(img.mul(img), variance, {kernel_size, kernel_size});

  cv::Mat stddev;
  cv::sqrt(variance - avg.mul(avg), stddev);

  return stddev;
}

cv::Mat mat2gray(const cv::Mat& src)
{
  cv::Mat dst;
  normalize(src, dst, 0.0, 1.0, cv::NORM_MINMAX);
  return dst;
}

cv::Matx33f getHomography(ps_image ref, ps_image target)
{
  cv::Ptr<cv::Feature2D> desc_extractor = cv::xfeatures2d::SURF::create();
  cv::BFMatcher matcher{desc_extractor->defaultNorm()};
  std::vector<std::vector<cv::DMatch>> matches;

  matcher.knnMatch(target.descriptors, ref.descriptors, matches, 2);
  std::vector<cv::DMatch> good_matches = extractGoodRatioMatches(matches, 0.8f);

  cv::Matx33f H_cv;
  if (good_matches.size() >= 10)
  {
    // Extract pixel coordinates for corresponding points.
    Eigen::Matrix2Xf matching_pts1;
    Eigen::Matrix2Xf matching_pts2;
    extractMatchingPoints(target.keypoints, ref.keypoints, good_matches, matching_pts1, matching_pts2);
    // Estimate homography.
    HomographyEstimator estimator;
    HomographyEstimate estimate = estimator.estimate(matching_pts1, matching_pts2);

    // Convert homography estimate to OpenCV matrix.
    cv::eigen2cv(estimate.homography, H_cv);
  }

  return H_cv;
}

void getRotationAndTranslation(const ps_image& ref, const ps_image& target, cv::Mat& R, cv::Mat& t)
{
  cv::Ptr<cv::Feature2D> desc_extractor = cv::xfeatures2d::SURF::create();
  cv::BFMatcher matcher{desc_extractor->defaultNorm()};
  std::vector<std::vector<cv::DMatch>> matches;

  matcher.knnMatch(target.descriptors, ref.descriptors, matches, 2);
  std::vector<cv::DMatch> good_matches = extractGoodRatioMatches(matches, 0.8f);

  if (good_matches.size() < 7)
  { throw std::runtime_error("Not enough point correspondences to find fundamental matrix"); }

  // Extract pixel coordinates for corresponding points.
  std::vector<cv::Point2f> matching_pts1;
  std::vector<cv::Point2f> matching_pts2;
  extractMatchingPointsCv(ref.keypoints, target.keypoints, good_matches, matching_pts1, matching_pts2);

  const auto K = ref.intrinsics.toCalibrationMatrix();
  cv::Mat K_cv;
  cv::eigen2cv(K, K_cv);

  const auto E = cv::findEssentialMat(matching_pts1, matching_pts2, K_cv);
  cv::recoverPose(E, matching_pts1, matching_pts2, K_cv, R, t);
}

void extractMatchingPointsCv(const std::vector<cv::KeyPoint>& keypts1, const std::vector<cv::KeyPoint>& keypts2,
                             const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& matched_pts1,
                             std::vector<cv::Point2f>& matched_pts2)
{
  matched_pts1.reserve(matches.size());
  matched_pts2.reserve(matches.size());

  for (size_t i = 0; i < matches.size(); ++i)
  {
    matched_pts1.emplace_back(cv::Point2f{keypts1[matches[i].queryIdx].pt.x, keypts1[matches[i].queryIdx].pt.y});
    matched_pts2.emplace_back(cv::Point2f{keypts2[matches[i].trainIdx].pt.x, keypts2[matches[i].trainIdx].pt.y});
  }
}

cv::Mat getPlainSweepHomography(const ps_image& ref, const ps_image& target, const cv::Vec3d& n, double d_m)
{
  cv::Ptr<cv::Feature2D> desc_extractor = cv::xfeatures2d::SURF::create();
  cv::BFMatcher matcher{desc_extractor->defaultNorm()};
  std::vector<std::vector<cv::DMatch>> matches;

  matcher.knnMatch(ref.descriptors, target.descriptors, matches, 2);
  std::vector<cv::DMatch> good_matches = extractGoodRatioMatches(matches, 0.8f);

  if (good_matches.size() < 5)
  { throw std::runtime_error("Not enough point correspondences to find Essential matrix"); }

  // Extract pixel coordinates for corresponding points.
  std::vector<cv::Point2f> matching_pts_ref;
  std::vector<cv::Point2f> matching_pts_target;
  extractMatchingPointsCv(ref.keypoints, target.keypoints, good_matches, matching_pts_ref, matching_pts_target);

  //Find essential matrix
  cv::Mat K_cv, mask;
  cv::eigen2cv(ref.intrinsics.toCalibrationMatrix(), K_cv);
  const auto E_cv = cv::findEssentialMat(matching_pts_ref, matching_pts_target, K_cv, CV_RANSAC, 0.999, 1.0, mask);

  //Decompose the Essential matrix into rotation and translation
  cv::Mat R, t;
  cv::recoverPose(E_cv, matching_pts_ref, matching_pts_target, K_cv, R, t, mask);

  //Get all matrixes as eigen matrixes
  Eigen::Matrix3d R_eigen;
  cv::cv2eigen(R, R_eigen);
//  const auto R_inv = R_eigen.inverse();
  Eigen::Vector3d t_eigen;
  cv::cv2eigen(t, t_eigen);
//  const auto t_inv = -R_inv * t_eigen;
  const Eigen::Vector3d n_eigen{n[0], n[1], n[2]};
  const auto K_ref = ref.intrinsics.toCalibrationMatrix();
  const auto K_target = ref.intrinsics.toCalibrationMatrix();

  const Eigen::Matrix3d H_m = K_target * (R_eigen - (t_eigen * (n_eigen.transpose() / d_m))) * K_ref.inverse();

  const Eigen::Matrix3d H_m_inv = H_m.inverse();
  cv::Mat H_m_cv;
  cv::eigen2cv(H_m_inv, H_m_cv);
  return H_m_cv;
}