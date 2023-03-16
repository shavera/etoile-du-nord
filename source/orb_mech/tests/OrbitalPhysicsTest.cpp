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

  // choosing something roughly physical/coplanar with left hand orbit
  const OrbitalPhysicsParameters ellipticalOrbitParams{
      StandardGravParam{1862},
      StateVector{
          PositionVector {{-6}, {18}, {-1}},
          VelocityVector {{12}, {4}, {3}}
      }
  };
  const OrbitalPhysics ellipticalOrbit{ellipticalOrbitParams};

  // orbit with no angular momentum that "stops" "at infinity"
  // e.g. the trajectory of a body "falling in" from "infinity"
  const OrbitalPhysicsParameters parabolicFFParams{
      StandardGravParam{1862},
      StateVector{
          PositionVector {{-6}, {18}, {-1}},
          VelocityVector {{}, {}, {}}
      }
  };
  const OrbitalPhysics parabolicFFOrbit{ellipticalOrbitParams};

  // orbit with no angular momentum that never asymptotically "stops"
  // e.g. an object "fired" toward the body with some high energy
  const OrbitalPhysicsParameters hyperbolicFFParams{
      StandardGravParam{1862},
      StateVector{
          PositionVector {{-6}, {18}, {-1}},
          VelocityVector {{12}, {4}, {3}}
      }
  };
  const OrbitalPhysics hyperbolicFFOrbit{ellipticalOrbitParams};

  // "elliptical" freefall orbit that does not have a trivial coordinate system
  // to check that we get reasonable stuff for angles


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
//    RadiansPerSecond sweep;
//    Angle inclination;
//    Angle longitudeOfAscendingNode;
//    Angle argumentOfPeriapsis;
//    double eccentricity;
  };

  const std::map<std::string, TestExpectations> testExpectationsMap{
      {"unitCircle", {OrbitalPhysics::Shape::elliptical, {1}, {2*kPi}}},
      {"freeFall", {OrbitalPhysics::Shape::elliptical, {0.5}, {kPi/sqrt(2)}}},
      {"parabolic", {OrbitalPhysics::Shape::parabolic, {std::nan("parabolic orbit - no semimajoraxis")}, std::numeric_limits<double>::infinity()}},
      {"hyperbolic", {OrbitalPhysics::Shape::hyperbolic, {-64.2068965517}, std::numeric_limits<double>::infinity()}},
      {"elliptical", {OrbitalPhysics::Shape::elliptical, {68.962962963}, {83.3899855854}}}

  };
};

TEST_P(OrbitalPhysicsTest, shape){

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

}

INSTANTIATE_TEST_SUITE_P(OrbitalPhysicsTest, OrbitalPhysicsTest,
                         Values("unitCircle", "freeFall", "parabolic", "hyperbolic", "elliptical"));

} // namespace
} // namespace orb_mech
