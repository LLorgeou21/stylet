#ifndef STYLET_PERCEPTION__REGISTRATION_UTILS_HPP_
#define STYLET_PERCEPTION__REGISTRATION_UTILS_HPP_

#include <vector>

#include <Eigen/Geometry>

namespace stylet_perception
{

// eigh/PCA doesn't guarantee det=+1 (it can return a reflection, not a true
// rotation) - this forces it by flipping the last axis if needed. Without
// this, the "4 valid sign combinations" below would actually be 4 reflections.
inline Eigen::Matrix3f ensureProperRotation(Eigen::Matrix3f axes)
{
  if (axes.determinant() < 0.0f)
  {
    axes.col(2) *= -1.0f;
  }
  return axes;
}

// The 4 sign combinations (out of 8 possible) that preserve a true rotation
// (determinant +1) rather than a reflection (determinant -1).
inline std::vector<Eigen::Vector3f> validSignFlips()
{
  std::vector<Eigen::Vector3f> flips;
  for (float sx : {1.0f, -1.0f})
    for (float sy : {1.0f, -1.0f})
      for (float sz : {1.0f, -1.0f})
        if (sx * sy * sz > 0.0f)
          flips.push_back(Eigen::Vector3f(sx, sy, sz));
  return flips;  // always exactly 4 elements
}

inline Eigen::Matrix4f transformFromFlip(
  const Eigen::Vector3f & flip, const Eigen::Matrix3f & observed_axes,
  const Eigen::Matrix3f & reference_axes, const Eigen::Vector4f & observed_centroid,
  const Eigen::Vector4f & reference_centroid)
{
  Eigen::Matrix3f candidate_axes = observed_axes;
  candidate_axes.col(0) *= flip[0];
  candidate_axes.col(1) *= flip[1];
  candidate_axes.col(2) *= flip[2];

  Eigen::Matrix3f R = candidate_axes * reference_axes.transpose();
  Eigen::Vector3f t = observed_centroid.head<3>() - R * reference_centroid.head<3>();

  Eigen::Matrix4f T_init = Eigen::Matrix4f::Identity();
  T_init.block<3, 3>(0, 0) = R;
  T_init.block<3, 1>(0, 3) = t;
  return T_init;
}

}  // namespace stylet_perception

#endif  // STYLET_PERCEPTION__REGISTRATION_UTILS_HPP_
