#include <gtest/gtest.h>

#include "stylet_planning/geometry_utils.hpp"

using stylet_planning::quaternionFromZAxis;
using stylet_planning::setAllowedCollisionPair;

namespace
{
// Checks that the quaternion's rotation matrix is orthonormal (columns unit
// length, mutually perpendicular, determinant +1 - a proper rotation, not a
// reflection).
void expectOrthonormalRotation(const Eigen::Matrix3d & R)
{
  Eigen::Matrix3d should_be_identity = R.transpose() * R;
  EXPECT_TRUE(should_be_identity.isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
}
}  // namespace

TEST(QuaternionFromZAxis, ZAxisMatchesInputDirection)
{
  Eigen::Vector3d direction(0.3, -0.8, 0.5);
  Eigen::Quaterniond q = quaternionFromZAxis(direction);
  Eigen::Matrix3d R = q.toRotationMatrix();

  expectOrthonormalRotation(R);
  EXPECT_TRUE(R.col(2).isApprox(direction.normalized(), 1e-9));
}

TEST(QuaternionFromZAxis, UnnormalizedInputStillWorks)
{
  // The insertion axis is always built from a difference of two points
  // (target - entry), never pre-normalized by the caller.
  Eigen::Vector3d direction(0.0, 0.0, 5.0);
  Eigen::Quaterniond q = quaternionFromZAxis(direction);
  Eigen::Matrix3d R = q.toRotationMatrix();

  expectOrthonormalRotation(R);
  EXPECT_TRUE(R.col(2).isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-9));
}

TEST(QuaternionFromZAxis, NearParallelToReferenceUsesFallback)
{
  // Direction almost exactly along the default reference (0, 0, 1) - exercises
  // the fallback branch (reference switched to (1, 0, 0)) that avoids a
  // degenerate cross product.
  Eigen::Vector3d direction(0.001, 0.0, 1.0);
  Eigen::Quaterniond q = quaternionFromZAxis(direction);
  Eigen::Matrix3d R = q.toRotationMatrix();

  expectOrthonormalRotation(R);
  EXPECT_TRUE(R.col(2).isApprox(direction.normalized(), 1e-9));
}

TEST(QuaternionFromZAxis, ExactlyAntiParallelToReferenceUsesFallback)
{
  Eigen::Vector3d direction(0.0, 0.0, -1.0);
  Eigen::Quaterniond q = quaternionFromZAxis(direction);
  Eigen::Matrix3d R = q.toRotationMatrix();

  expectOrthonormalRotation(R);
  EXPECT_TRUE(R.col(2).isApprox(direction.normalized(), 1e-9));
}

TEST(SetAllowedCollisionPair, AddsSymmetricEntryToEmptyMatrix)
{
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  setAllowedCollisionPair(acm, "needle_base", "target");

  ASSERT_EQ(acm.entry_names.size(), 2u);
  int needle_idx = acm.entry_names[0] == "needle_base" ? 0 : 1;
  int target_idx = 1 - needle_idx;

  EXPECT_TRUE(acm.entry_values[needle_idx].enabled[target_idx]);
  EXPECT_TRUE(acm.entry_values[target_idx].enabled[needle_idx]);
}

TEST(SetAllowedCollisionPair, PreservesExistingEntries)
{
  // Reproduces the exact bug this function was written to avoid: a first
  // implementation replaced the whole matrix with a partial diff, wiping out
  // the SRDF's own adjacent-link self-collision exemptions.
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  acm.entry_names = {"base_link_inertia", "shoulder_link"};
  moveit_msgs::msg::AllowedCollisionEntry entry_a;
  entry_a.enabled = {false, true};
  moveit_msgs::msg::AllowedCollisionEntry entry_b;
  entry_b.enabled = {true, false};
  acm.entry_values = {entry_a, entry_b};

  setAllowedCollisionPair(acm, "needle_base", "target");

  ASSERT_EQ(acm.entry_names.size(), 4u);
  int base_idx = 0;
  int shoulder_idx = 1;
  // The pre-existing exemption must survive untouched.
  EXPECT_TRUE(acm.entry_values[base_idx].enabled[shoulder_idx]);
  EXPECT_TRUE(acm.entry_values[shoulder_idx].enabled[base_idx]);
}

TEST(SetAllowedCollisionPair, DoesNotDuplicateExistingNames)
{
  moveit_msgs::msg::AllowedCollisionMatrix acm;
  setAllowedCollisionPair(acm, "needle_base", "target");
  setAllowedCollisionPair(acm, "needle_base", "target");

  EXPECT_EQ(acm.entry_names.size(), 2u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
