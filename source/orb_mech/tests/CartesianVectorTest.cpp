#include "orb_mech/CartesianVector.h"

#include <gtest/gtest.h>

namespace orb_mech {

namespace {

class CartesianVectorTest : public ::testing::Test {

public:
  const double expectedX{1.2345}, expectedY{-2.341}, expectedZ{3.0};
  const double expectedNorm{std::sqrt(std::pow(expectedX, 2) + std::pow(expectedY, 2) + std::pow(expectedZ, 2))};
  const CartesianVector vector{expectedX, expectedY, expectedZ};
};

TEST_F(CartesianVectorTest, basicGetters) {
  EXPECT_EQ(expectedX, vector.x());
  EXPECT_EQ(expectedY, vector.y());
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

TEST_F(CartesianVectorTest, scale){
  {
    SCOPED_TRACE("member operator (scalar on right)");
    const CartesianVector expectedVector{2.469, -4.682, 6.0};
    const CartesianVector scaledVector{vector * 2.0};
    EXPECT_EQ(expectedVector, scaledVector);
  }
  {
    SCOPED_TRACE("static global operator (scalar on left)");
    const CartesianVector expectedVector{-4.32075, 8.1935, -10.5};
    const CartesianVector scaledVector{-3.5 * vector};
    EXPECT_NEAR(expectedVector.x(), scaledVector.x(), 1e-8);
    EXPECT_NEAR(expectedVector.y(), scaledVector.y(), 1e-8);
    EXPECT_NEAR(expectedVector.z(), scaledVector.z(), 1e-8);
  }
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

TEST_F(CartesianVectorOperationsTest, equality){
  EXPECT_EQ(leftVector, leftVector);
  EXPECT_NE(leftVector, rightVector);
}

TEST_F(CartesianVectorOperationsTest, addition) {
  const CartesianVector expectedVector{3.61, 0.98, 0.95};

  const auto actualVector = leftVector + rightVector;
  EXPECT_LT(expectedVector.separation(actualVector), 1e-10);
}

TEST_F(CartesianVectorOperationsTest, subtraction) {
  const CartesianVector expectedVector{-1.15, -7.46, 3.63};

  const auto actualVector = leftVector - rightVector;
  // since we use subtraction in the 'separation' method below, want to check
  // each element individually rather than using separation to avoid circular test
  EXPECT_NEAR(expectedVector.x(), actualVector.x(), 1e-10);
  EXPECT_NEAR(expectedVector.y(), actualVector.y(), 1e-10);
  EXPECT_NEAR(expectedVector.z(), actualVector.z(), 1e-10);
}

TEST_F(CartesianVectorOperationsTest, separation) {
  const double expectedSeparation{8.3756193801};

  const double actualSeparation{leftVector.separation(rightVector)};
  EXPECT_NEAR(expectedSeparation, actualSeparation, 1e-10);
}

class VectorQuantityTest : public CartesianVectorTest{
public:
  struct ArbitraryUnitType{
    double someValue;
  };
  using ArbitraryVector = VectorQuantity<ArbitraryUnitType>;

  const ArbitraryVector arbitraryVector{{expectedX}, {expectedY}, {expectedZ}};
};

TEST_F(VectorQuantityTest, basicGetters) {
  {
      SCOPED_TRACE("Three unit constructor");
      EXPECT_EQ(expectedX, arbitraryVector.x().someValue);
      EXPECT_EQ(expectedY, arbitraryVector.y().someValue);
      EXPECT_EQ(expectedZ, arbitraryVector.z().someValue);
  }

  {
      SCOPED_TRACE("Vector initialized constructor");
      const ArbitraryVector vectorInitializedVector{vector};

      EXPECT_EQ(expectedX, vectorInitializedVector.x().someValue);
      EXPECT_EQ(expectedY, vectorInitializedVector.y().someValue);
      EXPECT_EQ(expectedZ, vectorInitializedVector.z().someValue);
  }
}

TEST_F(VectorQuantityTest, norm) {
  EXPECT_EQ(expectedNorm, arbitraryVector.norm().someValue);
}

TEST_F(VectorQuantityTest, normalizedVector) {
  ArbitraryVector normalizedVector{arbitraryVector.normalizedVector()};
  EXPECT_EQ(expectedX / expectedNorm, normalizedVector.x().someValue);
  EXPECT_EQ(expectedY / expectedNorm, normalizedVector.y().someValue);
  EXPECT_EQ(expectedZ / expectedNorm, normalizedVector.z().someValue);
}

TEST_F(VectorQuantityTest, equality){
  const ArbitraryVector otherVector{{1}, {2}, {3}};
  EXPECT_EQ(arbitraryVector, arbitraryVector);
  EXPECT_NE(arbitraryVector, otherVector);
}

TEST(StateVectorTest, distance) {
  const StateVector stateVector{PositionVector{{3},{-14}, {18}},
                                VelocityVector{{}, {}, {}}};
  // 3, 14, 18 = 23 pythagorean quadruple
  EXPECT_EQ(23, stateVector.distance.m);
}

TEST(StateVectorTest, speed) {
  const StateVector stateVector{PositionVector{{},{}, {}},
                                VelocityVector{{-12}, {16}, {21}}};
  // 12, 16, 21 = 29 pythagorean quadruple
  EXPECT_EQ(29, stateVector.speed.mps);
}

} // namespace
} // namespace orb_mech