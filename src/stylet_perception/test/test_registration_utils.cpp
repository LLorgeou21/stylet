#include <gtest/gtest.h>

#include "stylet_perception/registration_utils.hpp"

using stylet_perception::ensureProperRotation;
using stylet_perception::transformFromFlip;
using stylet_perception::validSignFlips;

TEST(EnsureProperRotation, LeavesAProperRotationUnchanged)
{
  Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f result = ensureProperRotation(identity);

  EXPECT_TRUE(result.isApprox(identity));
  EXPECT_NEAR(result.determinant(), 1.0f, 1e-6f);
}

TEST(EnsureProperRotation, FlipsAReflectionIntoAProperRotation)
{
  // A single-axis flip of the identity is a reflection (det = -1), exactly
  // the kind of result PCA's eigenvector decomposition can hand back.
  Eigen::Matrix3f reflection = Eigen::Matrix3f::Identity();
  reflection(0, 0) = -1.0f;
  ASSERT_NEAR(reflection.determinant(), -1.0f, 1e-6f);

  Eigen::Matrix3f result = ensureProperRotation(reflection);

  EXPECT_NEAR(result.determinant(), 1.0f, 1e-6f);
  // Only the last column should have been touched.
  EXPECT_TRUE(result.col(0).isApprox(reflection.col(0)));
  EXPECT_TRUE(result.col(1).isApprox(reflection.col(1)));
  EXPECT_TRUE(result.col(2).isApprox(-reflection.col(2)));
}

TEST(ValidSignFlips, ReturnsExactlyFourCombinations)
{
  std::vector<Eigen::Vector3f> flips = validSignFlips();
  EXPECT_EQ(flips.size(), 4u);
}

TEST(ValidSignFlips, EveryCombinationPreservesAProperRotation)
{
  // By construction, sx*sy*sz > 0 for every returned flip - applying one to
  // an orthonormal basis must keep its determinant positive.
  for (const auto & flip : validSignFlips())
  {
    float product = flip[0] * flip[1] * flip[2];
    EXPECT_GT(product, 0.0f);
    for (int i = 0; i < 3; ++i)
    {
      EXPECT_TRUE(std::abs(flip[i]) == 1.0f);
    }
  }
}

TEST(ValidSignFlips, AllFourCombinationsAreDistinct)
{
  std::vector<Eigen::Vector3f> flips = validSignFlips();
  for (size_t i = 0; i < flips.size(); ++i)
  {
    for (size_t j = i + 1; j < flips.size(); ++j)
    {
      EXPECT_FALSE(flips[i].isApprox(flips[j]));
    }
  }
}

TEST(TransformFromFlip, IdentityFlipWithMatchingAxesYieldsIdentityRotation)
{
  Eigen::Vector3f flip(1.0f, 1.0f, 1.0f);
  Eigen::Matrix3f axes = Eigen::Matrix3f::Identity();
  Eigen::Vector4f observed_centroid(1.0f, 2.0f, 3.0f, 1.0f);
  Eigen::Vector4f reference_centroid(0.0f, 0.0f, 0.0f, 1.0f);

  Eigen::Matrix4f T = transformFromFlip(flip, axes, axes, observed_centroid, reference_centroid);

  Eigen::Matrix3f rotation_part = T.block<3, 3>(0, 0);
  Eigen::Vector3f translation_part = T.block<3, 1>(0, 3);
  EXPECT_TRUE(rotation_part.isApprox(Eigen::Matrix3f::Identity()));
  EXPECT_TRUE(translation_part.isApprox(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));
}

TEST(TransformFromFlip, ResultIsAValidRigidTransform)
{
  Eigen::Vector3f flip(1.0f, -1.0f, -1.0f);  // one of the 4 valid flips
  Eigen::Matrix3f observed_axes = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f reference_axes = Eigen::Matrix3f::Identity();
  Eigen::Vector4f observed_centroid(0.5f, -0.2f, 0.1f, 1.0f);
  Eigen::Vector4f reference_centroid(0.0f, 0.0f, 0.0f, 1.0f);

  Eigen::Matrix4f T = transformFromFlip(
    flip, observed_axes, reference_axes, observed_centroid, reference_centroid);

  Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  EXPECT_NEAR((R.transpose() * R - Eigen::Matrix3f::Identity()).norm(), 0.0f, 1e-5f);
  EXPECT_NEAR(R.determinant(), 1.0f, 1e-5f);
  EXPECT_FLOAT_EQ(T(3, 3), 1.0f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
