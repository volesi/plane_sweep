#include "../include/plane_sweep/intrinsics.h"

Eigen::Matrix3d Intrinsics::toCalibrationMatrix()
{
  Eigen::Matrix3d calibration_matrix;
  calibration_matrix <<
                      fu, s, cu,
                      0, fv, cv,
                      0,  0,  1;
  // Todo: Construct the calibration matrix correctly.
  return calibration_matrix;
}

Intrinsics::Vector5d Intrinsics::toDistortionCoefficientVector()
{
  Vector5d return_value;
  return_value << k1, k2, 0, 0, k3;
  // Todo: Construct the distortion coefficients on the form [k1, k2, 0, 0, k3].
  return return_value;
}

std::istream& operator>>(std::istream& is, Intrinsics& intrinsics)
{
  is >> intrinsics.fu
     >> intrinsics.fv
     >> intrinsics.s
     >> intrinsics.cu
     >> intrinsics.cv
     >> intrinsics.k1
     >> intrinsics.k2
     >> intrinsics.k3;

  if (!is)
  {
    throw std::runtime_error("Could not read Intrinsics data");
  }

  return is;
}

std::ostream& operator<<(std::ostream& os, const Intrinsics& intrinsics)
{
  os << "fu: " << intrinsics.fu << "\n"
     << "fv: " << intrinsics.fv << "\n"
     << " s: " << intrinsics.s << "\n"
     << "cu: " << intrinsics.cu << "\n"
     << "cv: " << intrinsics.cv << "\n"
     << "k1: " << intrinsics.k1 << "\n"
     << "k2: " << intrinsics.k2 << "\n"
     << "k3: " << intrinsics.k3 << "\n";

  return os;
}
