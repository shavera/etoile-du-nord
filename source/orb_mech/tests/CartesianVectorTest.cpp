#include "orb_mech/CartesianVector.h"

#include <gtest/gtest.h>

namespace orb_mech {
namespace {

class CartesianVectorTest : public ::testing::Test {

public:
  const double expectedX{1.2345}, expectedY{-2.341}, expectedZ{3.0};
  const double expectedNorm{std::sqrt(std::pow(expectedX, 2) + std::pow(expectedY, 2) + std::pow(expectedZ, 2))};
  CartesianVector vector{expectedX, expectedY, expectedZ};
};

TEST_F(CartesianVectorTest, x) {
  EXPECT_EQ(expectedX, vector.x());
}

TEST_F(CartesianVectorTest, y) {
  EXPECT_EQ(expectedY, vector.y());
}

TEST_F(CartesianVectorTest, z) {
  EXPECT_EQ(expectedZ, vector.z());
}

TEST_F(CartesianVectorTest, norm) {
  EXPECT_EQ(expectedNorm, vector.norm());
}

TEST_F(CartesianVectorTest, normalizedVector) {
  CartesianVector normalizedVector{vector.normalizedVector()};
  EXPECT_EQ(expectedX / expectedNorm, normalizedVector.x());
  EXPECT_EQ(expectedY / expectedNorm, normalizedVector.y());
  EXPECT_EQ(expectedZ / expectedNorm, normalizedVector.z());
}

class CartesianVectorOperationsTest : public ::testing::Test {
public:
  const CartesianVector leftVector{1.23, -3.24, 2.29};
  const CartesianVector rightVector{2.38, 4.22, -1.34};
};

TEST_F(CartesianVectorOperationsTest, dot) {
  const double expectedValue{-13.814};
  EXPECT_EQ(expectedValue, CartesianVector::dot(leftVector, rightVector));
  EXPECT_EQ(expectedValue, leftVector.dot(rightVector));
}

TEST_F(CartesianVectorOperationsTest, cross) {
  const CartesianVector expectedVector{-5.3222, 7.0984, 12.9018};
  CartesianVector actualVector{CartesianVector::cross(leftVector, rightVector)};
  EXPECT_NEAR(expectedVector.x(), actualVector.x(), 1e-4);
  EXPECT_NEAR(expectedVector.y(), actualVector.y(), 1e-4);
  EXPECT_NEAR(expectedVector.z(), actualVector.z(), 1e-4);

  actualVector = leftVector.cross(rightVector);
  EXPECT_NEAR(expectedVector.x(), actualVector.x(), 1e-4);
  EXPECT_NEAR(expectedVector.y(), actualVector.y(), 1e-4);
  EXPECT_NEAR(expectedVector.z(), actualVector.z(), 1e-4);
}

} // namespace
} // namespace orb_mech