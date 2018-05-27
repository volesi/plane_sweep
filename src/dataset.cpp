#include "../include/plane_sweep/dataset.h"
#include "opencv2/imgcodecs.hpp"
#include <fstream>

bool Dataset::getNext(DataElement& element)
{
  get_succeeded_ = false;

  if (hasNext())
  {
    readImage(element);
    readMetadata(element);

    ++curr_num_list_index_;
    get_succeeded_ = true;
  }

  return bool(this);
}

bool Dataset::hasNext() const
{
  return curr_num_list_index_ < file_num_list_.size();
}

Dataset::operator bool() const
{
  return get_succeeded_;
}

void Dataset::readImage(DataElement& element) const
{
  std::stringstream filepath;
  filepath << folder_name_ << "110608_Oslo_0" << file_num_list_[curr_num_list_index_] << ".jpg";

  element.image = cv::imread(filepath.str());

  if (element.image.empty())
  {
    throw std::runtime_error("Could not open image file: " + filepath.str());
  }
}

void Dataset::readMetadata(DataElement& element) const
{
  element.img_num = file_num_list_[curr_num_list_index_];

  std::stringstream filepath;
  filepath << folder_name_ << "110608_Oslo_0" << file_num_list_[curr_num_list_index_] << ".txt";
  std::ifstream input_file{filepath.str()};

  if (!input_file)
  { throw std::runtime_error("Could not open file"); }

  input_file >> element.intrinsics
             >> element.camera_position_in_body
             >> element.camera_attitude_in_body
             >> element.body_position_in_geo
             >> element.body_attitude_in_geo;
}

Dataset& operator>>(Dataset& dataset, DataElement& element)
{
  dataset.getNext(element);

  return dataset;
}
