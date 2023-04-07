#include "orb_mech/units.h"

#include "gtest/gtest.h"

namespace orb_mech {
namespace {

using namespace ::testing;

constexpr double kPi = std::numbers::pi;

struct AngleTestDatum {
  double inputRadians;
  double inputDegrees;
  double expectedRadians;
  double expectedDegrees;
  std::string caseName;
};

class AngleTests : public TestWithParam<AngleTestDatum> {};

TEST_P(AngleTests, fromRadians) {
  const auto& datum = GetParam();
  Angle angle{Angle::radians(datum.inputRadians)};

  EXPECT_EQ(datum.expectedRadians, angle.getRadians());
  EXPECT_EQ(datum.expectedDegrees, angle.getDegrees());
  EXPECT_EQ(Angle::degrees(datum.expectedDegrees), angle);
}

TEST_P(AngleTests, fromDegrees) {
  const auto& datum = GetParam();
  Angle angle{Angle::degrees(datum.inputDegrees)};

  EXPECT_EQ(datum.expectedRadians, angle.getRadians());
  EXPECT_EQ(datum.expectedDegrees, angle.getDegrees());
  EXPECT_EQ(Angle::radians(datum.expectedRadians), angle);
}

constexpr double kQuarterPi = kPi / 4.0;

const std::vector<AngleTestDatum> angleTestData{
    {0, 0, 0, 0, "Zeroes"},
    {kQuarterPi, 45, kQuarterPi, 45, "QuarterPi"},
    {-kQuarterPi, -45, -kQuarterPi, -45, "NegativeQuarterPi"},
    {3.0 * kPi / 2.0, 270, -kPi / 2.0, -90, "ThreeHalvesPiWraparound"},
    {2.0 * kPi, 360.0, 0.0, 0.0, "TwoPiWraparound"}};

INSTANTIATE_TEST_SUITE_P(AngleTests,
                         AngleTests,
                         ValuesIn(angleTestData),
                         [](const auto& info) { return info.param.caseName; });

TEST(AngleZeroTest, zeroAngle) {
  const Angle zero{Angle::Zero()};
  EXPECT_EQ(0.0, zero.getRadians());
  EXPECT_EQ(0.0, zero.getDegrees());
}

}  // namespace
}  // namespace orb_mech
