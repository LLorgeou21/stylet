#ifndef STYLET_PLANNING__GEOMETRY_UTILS_HPP_
#define STYLET_PLANNING__GEOMETRY_UTILS_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/msg/allowed_collision_entry.hpp"

namespace stylet_planning
{

// Builds a full rotation from a single desired Z axis (a vector only fixes 2 of 3
// degrees of freedom - a free rotation remains around that axis). An arbitrary
// reference vector completes the orthonormal basis (X, Y, Z) via cross products,
// with a fallback reference if Z is nearly parallel to it (otherwise the cross
// product degenerates).
inline Eigen::Quaterniond quaternionFromZAxis(const Eigen::Vector3d & z_axis_in)
{
  Eigen::Vector3d z_axis = z_axis_in.normalized();
  Eigen::Vector3d up_ref(0.0, 0.0, 1.0);
  if (std::abs(z_axis.dot(up_ref)) > 0.99)
  {
    up_ref = Eigen::Vector3d(1.0, 0.0, 0.0);
  }
  Eigen::Vector3d x_axis = up_ref.cross(z_axis).normalized();
  Eigen::Vector3d y_axis = z_axis.cross(x_axis);

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;

  return Eigen::Quaterniond(rotation);
}

// Adds the pair (a, b) to the allowed-collision matrix, preserving all
// existing entries (unlike a partial diff, which replaces the whole matrix -
// see procedure_planner.cpp's allowNeedleTargetCollision() for the full story).
inline void setAllowedCollisionPair(
  moveit_msgs::msg::AllowedCollisionMatrix & acm,
  const std::string & a, const std::string & b)
{
  auto ensure_index = [&acm](const std::string & name) -> size_t
  {
    auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
    if (it != acm.entry_names.end())
    {
      return static_cast<size_t>(std::distance(acm.entry_names.begin(), it));
    }
    // New entry: extend every existing row by one column (default value
    // false = collision normally checked), then append the new row itself.
    for (auto & entry : acm.entry_values)
    {
      entry.enabled.push_back(false);
    }
    acm.entry_names.push_back(name);
    moveit_msgs::msg::AllowedCollisionEntry new_entry;
    new_entry.enabled = std::vector<bool>(acm.entry_names.size(), false);
    acm.entry_values.push_back(new_entry);
    return acm.entry_names.size() - 1;
  };

  size_t i = ensure_index(a);
  size_t j = ensure_index(b);
  acm.entry_values[i].enabled[j] = true;
  acm.entry_values[j].enabled[i] = true;
}

}  // namespace stylet_planning

#endif  // STYLET_PLANNING__GEOMETRY_UTILS_HPP_
