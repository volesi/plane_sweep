#ifndef PLANE_SWEEP_GEOMETRY_LAB_1_2_H
#define PLANE_SWEEP_GEOMETRY_LAB_1_2_H

#include <string>

/// \brief Main Lab program.
class Lab_1_2
{
public:
  /// \brief Constructs the lab.
  /// \param data_path Optional path to dataset.
  explicit Lab_1_2(const std::string& data_path = "../data/");

  /// \brief Runs the lab.
  void run();

private:
  std::string data_path_;
  std::string window_name_;
};

#endif //PLANE_SWEEP_GEOMETRY_LAB_1_2_H
