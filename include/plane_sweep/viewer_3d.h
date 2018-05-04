#ifndef PLANE_SWEEP_GEOMETRY_VIEWER_3D_H
#define PLANE_SWEEP_GEOMETRY_VIEWER_3D_H

#include "perspective_camera_model.h"
#include "sophus/se3.hpp"
#include "opencv2/viz.hpp"

class Viewer3D
{
public:
  Viewer3D();

  void addBody(Sophus::SE3d pose_local_body, int img_num);
  void addCamera(const PerspectiveCameraModel& cameraModel, const cv::Mat& img, int img_num);

  void spinOnce();
  void spin();

private:
  cv::viz::Viz3d viz_win_;
};

#endif //PLANE_SWEEP_GEOMETRY_VIEWER_3D_H
