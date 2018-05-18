#ifndef PLANE_SWEEP_GEOMETRY_PROJECT
#define PLANE_SWEEP_GEOMETRY_PROJECT

#include <string>

/// \brief Main Plane sweep project program.
class Project
{
public:
  /// \brief Constructs the project.
  /// \param data_path Optional path to dataset.
  explicit Project(const std::string& data_path = "../data/");

  /// \brief Runs the lab.
  void run();

private:
  std::string data_path_;
  std::string window_name_;
};

#endif //PLANE_SWEEP_GEOMETRY_PROJECT
