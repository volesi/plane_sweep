#include "../include/plane_sweep/attitude.h"
#include "Eigen/Geometry"

Eigen::Quaterniond Attitude::toQuaternion() const
{
  auto aa = Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(y_rot, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(x_rot, Eigen::Vector3d::UnitX());
  // Todo: Implement this correctly using Eigen::AngleAxisd (Z->Y>X).
  return aa;
}

Sophus::SO3d Attitude::toSO3() const
{
  return Sophus::SO3d(toQuaternion());
}

std::istream& operator>>(std::istream& is, Attitude& att)
{
  is >> att.x_rot
     >> att.y_rot
     >> att.z_rot;

  if (!is)
  {
    throw std::runtime_error("Could not read Attitude data");
  }

  return is;
}

std::ostream& operator<<(std::ostream& os, const Attitude& att)
{
  os << "x_rot: " << att.x_rot << "\n"
     << "y_rot: " << att.y_rot << "\n"
     << "z_rot: " << att.z_rot << "\n";

  return os;
}
