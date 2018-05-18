#include "../include/plane_sweep/project.h"

#include "../include/plane_sweep/dataset.h"
#include "../include/plane_sweep/local_coordinate_system.h"
#include "../include/plane_sweep/viewer_3d.h"
#include "../include/plane_sweep/plane_sweep.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

Project::Project(const std::string& data_path)
  : data_path_{data_path}
  , window_name_{"World point in camera"}
{}

void Project::run()
{
  // Set up dataset.
  Dataset dataset{data_path_};

  // Define local coordinate system based on the position of a light pole.
  const GeodeticPosition light_pole_position{59.963516, 10.667307, 321.0};
  const LocalCoordinateSystem local_system(light_pole_position);

  // Construct viewers.
  cv::namedWindow(window_name_);
  Viewer3D viewer;

  for (DataElement element{}; dataset >> element;)
  {
    PlaneSweep test;
    const auto& im = element.image;
    std::cout<< test.ZNCC(im, im, {im.rows/2, im.rows/2}, {im.rows/2 + 1, im.rows/2 + 1}, 10) << std::endl;
  }
  return;
  // Process each image in the dataset.
  for (DataElement element{}; dataset >> element;)
  {
    // Compute the pose of the body in the local coordinate system.
    // Todo: Finish Attitude::toQuaternion(). Done
    const Sophus::SE3d pose_local_body = local_system.toLocalPose(element.body_position_in_geo,
                                                                  element.body_attitude_in_geo.toSO3());

    // Add body coordinate axis to the 3D viewer.
    // Todo: Write line of code below to add body to viewer. done
    viewer.addBody(pose_local_body,element.img_num);

    // Compute the pose of the camera relative to the body.
    // Todo: Finish CartesianPosition::toVector(). done
    // Todo: Construct pose_body_camera correctly using element. done
    const Sophus::SE3d pose_body_camera{element.camera_attitude_in_body.toSO3(),
                                        element.camera_position_in_body.toVector()};

    // Compute the pose of the camera relative to the local coordinate system.
    // Todo: Construct pose_local_camera correctly using the poses above.
    const Sophus::SE3d pose_local_camera{pose_local_body*pose_body_camera};

    // Construct a camera model based on the intrinsic calibration and camera pose.
    // Todo: Finish Intrinsics::toCalibrationMatrix(). done
    // Todo: Finish Intrinsics::toDistortionVector(). done
    const PerspectiveCameraModel cameraModel{element.intrinsics.toCalibrationMatrix(),
                                             pose_local_camera,
                                             element.intrinsics.toDistortionCoefficientVector()};

    // Undistort image.
    // Todo: Undistort image using the camera model. Why should we undistort the image? done
    cv::Mat undistorted_img = cameraModel.undistortImage(element.image);

    // Project world point (the origin) into the image.
    // Todo: Finish PerspectiveCameraModel::computeCameraProjectionMatrix(). done
    // Todo: Finish PerspectiveCameraModel::projectWorldPoint(). done
    // Todo: Optionally finish PerspectiveCameraModel::projectWorldPoints().
    const Eigen::Vector2d pix_pos = cameraModel.projectWorldPoint(Eigen::Vector3d::Zero());

    // Draw a marker in the image at the projected position.
    const Eigen::Vector2i pix_pos_int = (pix_pos.array().round()).cast<int>();
    cv::drawMarker(undistorted_img, {pix_pos_int.x(), pix_pos_int.y()}, {0.,0.,255.}, cv::MARKER_CROSS, 40, 3);

    // Show the image.
    // Todo: Write line of code below to show the image with the marker.
    cv::imshow("World point in camera", undistorted_img);

    // Add the camera to the 3D viewer.
    // Todo: Write line of code below to add body to viewer.
    viewer.addCamera(cameraModel,undistorted_img,element.img_num);
    // Update the windows.
    viewer.spinOnce();
    cv::waitKey(100);
  }

  // Remove image viewer.
  cv::destroyWindow(window_name_);

  // Run 3D viewer until stopped.
  viewer.spin();
}
