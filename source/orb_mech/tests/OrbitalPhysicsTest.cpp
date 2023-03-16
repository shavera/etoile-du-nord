#include "../OrbitalPhysics.h"

#include "gtest/gtest.h"
#include "gmock/gmock-matchers.h"

#include <cmath>
#include <numbers>

namespace orb_mech{
namespace {

using namespace ::testing;

const auto kPi = std::numbers::pi;

class OrbitalPhysicsTest : public TestWithParam<std::string>{
public:
  // should produce a trivial circular orbit
  const OrbitalPhysicsParameters unitCircleParams{
    StandardGravParam{1},
        StateVector{
      PositionVector {{1},{},{}},
        VelocityVector {{}, {1}, {}}
    }};
  const OrbitalPhysics unitCircleOrbit{unitCircleParams};

  // should produce a trivial "orbit" which falls straight toward center
  // this actually forms an edge case to test a lot of orbital extrema
  const OrbitalPhysicsParameters unitFreeFallParams{
      StandardGravParam{1},
      StateVector{
          PositionVector {{1},{},{}},
          VelocityVector {{},{},{}}
      }
  };
  const OrbitalPhysics unitFreeFallOrbit{unitFreeFallParams};

  // for nontrivial orbits, reusing similar maths to phys params test:
  // parabolic orbit with speed = 14, elliptical with speed = 13, hyperbolic 15
  // Then choosing a radial distance of 19.0 which can be decomposed into
  // 1, 6, 18 to choose some alternate positions to start
  // for parabolic, 14 doesn't decompose easily, but will put it all along one
  // axis to allow for simplifications of hand calculations to double check
  // selecting grav param = 19 * 98 = 1862, which should cancel everything out
  const OrbitalPhysicsParameters parabolicOrbitParams{
      StandardGravParam{1862},
      StateVector{
          PositionVector {{6}, {-18}, {1}},
          VelocityVector {{14}, {}, {}}
      }
  };
  // ang mom = (0, 14, -252) = 70*sqrt(13); h^2 = 63700; norm = (0, 1/(5sqrt(13), 18/(5sqrt(13))
  // rp = h^2 / 2*mu = 17.1052631579; M=sqrt(mu/(2*rp^3))=0.4313007372
  // incl = acos(18/5sqrt(13)) = 3.1798301199 degrees
  // longAscNode = atan2(hx, -hy) = atan2(0, 14) = pi
  const OrbitalPhysics parabolicOrbit{parabolicOrbitParams};

  // choosing something roughly physical/coplanar with normal right hand orbit
  const OrbitalPhysicsParameters hyperbolicOrbitParams{
      StandardGravParam{1862},
      StateVector{
          PositionVector {{18}, {6}, {1}},
          VelocityVector {{-10}, {11}, {-2}}
      }
  };
  const OrbitalPhysics hyperbolicOrbit{hyperbolicOrbitParams};
  // ang mom = {-23, 26, 258} = 13*sqrt(401); norm = (-23/13sqrt(401), 2/sqrt(401), 258/13sqrt(401))
  // incl = acos(258/13sqrt(401)) = 7.6629523069 degrees
  // longAscNode = atan2(-23, -26) = -2.417342652841646 rad

  // choosing something roughly physical/coplanar with left hand orbit
  const OrbitalPhysicsParameters ellipticalOrbitParams{
      StandardGravParam{1862},
      StateVector{
          PositionVector {{-6}, {18}, {-1}},
          VelocityVector {{12}, {4}, {3}}
      }
  };
  const OrbitalPhysics ellipticalOrbit{ellipticalOrbitParams};
  // ang mom = {58, 6, -240} = 10*sqrt(610); norm = (29/5sqrt(610), 3/5sqrt(610), -12*sqrt(2/305))
  // incl = acos(-12*sqrt(2/305))=166.3442145528 degrees - corresponds to left hand orbit choice
  // longAscNode = atan2(58, -6) = 1.6738779353175968

  const std::map<std::string, const OrbitalPhysics&> testCaseMap{
      {"unitCircle", unitCircleOrbit},
      {"freeFall", unitFreeFallOrbit},
      {"parabolic", parabolicOrbit},
      {"hyperbolic", hyperbolicOrbit},
      {"elliptical", ellipticalOrbit}
  };

  struct TestExpectations{
    OrbitalPhysics::Shape shape;
    Meters semiMajorAxis;
    Seconds period;
    RadiansPerSecond sweep;
    Angle inclination;
    Angle longitudeOfAscendingNode;
//    Angle argumentOfPeriapsis;
//    double eccentricity;
  };

  const std::map<std::string, TestExpectations> testExpectationsMap{
      {"unitCircle",
       {OrbitalPhysics::Shape::elliptical,
        {1},
        {2*kPi},
        {1},
        {Angle::Zero()},
        {Angle::Zero()}
       }},
      {"freeFall",
       {OrbitalPhysics::Shape::elliptical,
        {0.5},
        {kPi/sqrt(2)},
        {2*sqrt(2)},
        {Angle::radians(std::nan("radial orbit - no inclination"))},
        {Angle::Zero()}
       }},
      {"parabolic",
       {OrbitalPhysics::Shape::parabolic,
        {std::nan("parabolic orbit - no semiMajor Axis")},
        {std::numeric_limits<double>::infinity()},
        {0.4313007372},// subtly different meaning of "sweep" for parabolic orbits
        {Angle::degrees(3.1798301199)},
        {Angle::radians(kPi)}
       }},
      {"hyperbolic",
        {OrbitalPhysics::Shape::hyperbolic,
         {-64.2068965517},
         {std::numeric_limits<double>::infinity()},
         {0.083872062},
        {Angle::degrees(7.6629523069)},
        {Angle::radians(-2.417342652841646)}
       }},
      {"elliptical",
       {OrbitalPhysics::Shape::elliptical,
        {68.962962963},
        {83.3899855854},
        {0.0753470008},
        {Angle::degrees(166.3442145528)},
        {Angle::radians(1.6738779353175968)}
       }}
  };
};

TEST_P(OrbitalPhysicsTest, shape){
  const OrbitalPhysics& orbit = testCaseMap.at(GetParam());
  const OrbitalPhysics::Shape shape = testExpectationsMap.at(GetParam()).shape;

  EXPECT_EQ(shape, orbit.shape());
}

TEST_P(OrbitalPhysicsTest, semiMajorAxis){
  const OrbitalPhysics& orbit = testCaseMap.at(GetParam());
  const Meters& expectedSemiMajorAxis = testExpectationsMap.at(GetParam()).semiMajorAxis;

  const auto actualSemiMajorAxis = orbit.semiMajorAxis();

  if("parabolic" == GetParam()){
    EXPECT_THAT(actualSemiMajorAxis.m, IsNan());
    return;
  }

  // sidenote: I'm still hung up on the freeFall semimajor axis point. My intuition
  // is that this number should be "1", not "0". imagine dropping an object from height 1,
  // it should fall "through" the body, continue on to height -1, then fall back "up" to
  // 1 once more, making the total "orbit" 2 long, and the semimajor of it 1. Need to think more

  EXPECT_NEAR(expectedSemiMajorAxis.m, actualSemiMajorAxis.m, 1e-10);
}

TEST_P(OrbitalPhysicsTest, period){
  const OrbitalPhysics& orbit = testCaseMap.at(GetParam());
  const Seconds expectedPeriod = testExpectationsMap.at(GetParam()).period;

  const auto actualPeriod = orbit.period();

  if("parabolic" == GetParam() || "hyperbolic" == GetParam()){
    EXPECT_TRUE(std::isinf(actualPeriod.s));
    return;
  }

  EXPECT_NEAR(expectedPeriod.s, actualPeriod.s, 1e-9);
}

TEST_P(OrbitalPhysicsTest, sweep){
  const OrbitalPhysics& orbit = testCaseMap.at(GetParam());
  const RadiansPerSecond expectedSweep = testExpectationsMap.at(GetParam()).sweep;

  const auto actualSweep = orbit.sweep();

  EXPECT_NEAR(expectedSweep.w, actualSweep.w, 1e-9);
}

TEST_P(OrbitalPhysicsTest, inclination){
  const OrbitalPhysics& orbit = testCaseMap.at(GetParam());
  const Angle expectedInclination = testExpectationsMap.at(GetParam()).inclination;

  const auto actualInclination = orbit.inclination();

  if("freeFall" == GetParam()){
    EXPECT_THAT(actualInclination.getDegrees(), IsNan());
    return;
  }

  // note: this comparison will do very poorly around ± pi radians, but probably not a problem in this test.
  EXPECT_NEAR(expectedInclination.getDegrees(), actualInclination.getDegrees(), 1e-9);
}

TEST_P(OrbitalPhysicsTest, longitudeOfAscendingNode){
  const OrbitalPhysics& orbit = testCaseMap.at(GetParam());
  const Angle expectedLongAscNode = testExpectationsMap.at(GetParam()).longitudeOfAscendingNode;

  const auto actualLongAscNode = orbit.longitudeOfAscendingNode();

  Angle angularDifference= expectedLongAscNode - actualLongAscNode;

  EXPECT_LT(std::fabs(angularDifference.getDegrees()), 0.1);
}

INSTANTIATE_TEST_SUITE_P(OrbitalPhysicsTest, OrbitalPhysicsTest,
                         Values("unitCircle", "freeFall", "parabolic", "hyperbolic", "elliptical"));

} // namespace
} // namespace orb_mech
