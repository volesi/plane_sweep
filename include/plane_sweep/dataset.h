#ifndef PLANE_SWEEP_GEOMETRY_DATASET_H
#define PLANE_SWEEP_GEOMETRY_DATASET_H

#include "attitude.h"
#include "cartesian_position.h"
#include "geodetic_position.h"
#include "intrinsics.h"

#include "opencv2/core.hpp"

/// \brief A set of data for each image in the dataset.
struct DataElement
{
  int img_num;
  cv::Mat image;
  Intrinsics intrinsics;
  CartesianPosition camera_position_in_body;
  Attitude camera_attitude_in_body;
  GeodeticPosition body_position_in_geo;
  Attitude body_attitude_in_geo;
};

/// \brief Represents the dataset for Lab 1.
class Dataset
{
public:
  /// \brief Constructs Dataset for planesweep algorithm
  /// \param folder_name the folder where the dataset resides.
  /// \param Vector of integers where the integers are the image numbers
  explicit Dataset(const std::string& folder_name, std::vector<int> file_num_list)
      : folder_name_(folder_name)
      , file_num_list_{file_num_list}
      , get_succeeded_{true}
      , curr_num_list_index_{0}
  {}

  /// \brief Reads the next data element.
  /// \param[out] element Data is read into this element.
  /// \return True if success.
  bool getNext(DataElement& element);

  /// \brief Checks if there is more data.
  /// \return True if more data.
  bool hasNext() const;

  /// \brief Checks if last get operation succeeded.
  /// \return True if success.
  operator bool() const;

private:
  void readImage(DataElement& element) const;

  void readMetadata(DataElement& element) const;

  std::string folder_name_;
  bool get_succeeded_;

  std::vector<int> file_num_list_;
  int curr_num_list_index_;
};

/// \brief Stream operator for getting the next data element.
Dataset& operator>>(Dataset& dataset, DataElement& element);

#endif //PLANE_SWEEP_GEOMETRY_DATASET_H
