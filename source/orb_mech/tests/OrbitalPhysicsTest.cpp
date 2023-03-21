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
  // ascNodeVec = {-14, 0, 0} -> Norm = {-1, 0, 0}
  // longAscNode = atan2(hx, -hy) = atan2(0, 14) = pi
  // Vxh = {0, -14*-252, 14*14} = {0, -3528, 196} ; Vxh/mu = {0, -36/19, 2/19}
  // r_hat = {6/19, -18/19, 1/19}
  // eccVec = {-6/19, (-36+18)/19, (2-1)/19} = {-6/19, -18/19, 1/19}
  // eccVecNorm -> same as {-6, -18, 1}, which has norm 19, therefore is already normalized (should be, parabolic case ecc=1)
  // argPeri -> acos(6/19) = 71.5915198294°
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
  // ang mom = {-23, 26, 258} = 13*sqrt(401); norm = (-23/13sqrt(401), 26/13sqrt(401), 258/13sqrt(401))
  // incl = acos(258/13sqrt(401)) = 7.6629523069 degrees
  // ascNodeVec = {-26, -23, 0} -> Norm = {-26/sqrt(1205), -23/sqrt(1205), 0}
  // longAscNode = atan2(-23, -26) = -2.417342652841646 rad
  // Vxh = {(11*258)-(-2*26), (-2*-23)-(-10*258), (-10*26)-(11*-23)} = {2890, 2626, -7}
  // r_hat = {18/19, 6/19, 1/19} = {1764/1862, 588/1862, 98/1862}
  // eccVec = {(2890-1764)/1862, (2626-588)/1862, (-7-98)/1862} = {1126/1862, 2038/1862, -105/1862} (norm = sqrt(5432345)/1862 = 1.25054... (ecc)
  // eccVecNorm same as {1126, 2038, -105} -> (1/sqrt(5432345))*{2548, 2518, -105}
  // argPeri = acos((-26*1126 + -23*2038 )/(sqrt(1205)*sqrt(5432345)) = acos(-76150/(sqrt(1205*5432345)) = 160.2543589611°
  const double hyperbolicEcc = hyperbolicOrbitParams.eccentricityVector().norm();

  // choosing something roughly physical/coplanar with left hand orbit
  const OrbitalPhysicsParameters ellipticalOrbitParams{
      StandardGravParam{1862},
      StateVector{
          PositionVector {{-6}, {18}, {-1}},
          VelocityVector {{12}, {4}, {3}}
      }
  };
  const OrbitalPhysics ellipticalOrbit{ellipticalOrbitParams};
  // ang mom = {58, 6, -240} = 10*sqrt(610); norm = (29/5sqrt(610), 3/5sqrt(610), -120/5sqrt(610)))
  // incl = acos(-12*sqrt(2/305))=166.3442145528 degrees - corresponds to left hand orbit choice
  // ascNodeVec = {-6, 58, 0} -> Norm = {-6/(10*sqrt(34)), 58/(10*sqrt(34)), 0} = {-3/5sqrt(34), 29/5sqrt(34, 0}
  // longAscNode = atan2(58, -6) = 1.6738779353175968
  // Vxh = {(4*-240)-(3*6), (3*58)-(12*-240), (12*6)-(4*58)} = {-978, 3054, -160}
  // r_hat = {-6/19, 18/19, -1/19} = {-588/1862, 1764/1862, -98/1862}
  // eccVec = {(-978+588)/1862, (3054-1764)/1862, (-160+98)/1862} = (1/1862)*{-390, 1290, -62} = (1/931){-195, 645, -31} - ecc = 0.7245381653
  // eccVecNorm same as {-195, 645, -31} -> (1/sqrt(455011))*{-195, 645, -31}
  // argPeri = acos((-195*-3+645*29)/(5*sqrt(34)*sqrt(455011)) = acos(19290/(5*sqrt(34*455011)) = acos(3858/sqrt(15470374)) = 11.2248489655°
  const double ellipticalEcc = ellipticalOrbitParams.eccentricityVector().norm();

  const std::map<std::string, const OrbitalPhysicsParameters&> paramsMap{
      {"unitCircle", unitCircleParams},
      {"freeFall", unitFreeFallParams},
      {"parabolic", parabolicOrbitParams},
      {"hyperbolic", hyperbolicOrbitParams},
      {"elliptical", ellipticalOrbitParams}
  };

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
    double eccentricity;
    Seconds period;
    RadiansPerSecond sweep;
    Angle inclination;
    Angle longitudeOfAscendingNode;
    Angle argumentOfPeriapsis;
  };

  const std::map<std::string, TestExpectations> testExpectationsMap{
      {"unitCircle",
       {OrbitalPhysics::Shape::elliptical,
        {1},
        0.0, // circular orbits are ecc = 0
        {2*kPi},
        {1},
        {Angle::Zero()},
        {Angle::Zero()}, // arbitrary for planar circle, selected to be zero
        {Angle::Zero()} // arbitrary for planar circle, selected to be zero
       }},
      {"freeFall",
       {OrbitalPhysics::Shape::elliptical, // technically elliptical because negative energy
        {0.5},
        1.0, // all radial orbits are ecc = 1
        {kPi/sqrt(2)},
        {2*sqrt(2)},
        {Angle::Zero()}, // known wrong, return to this.
        {Angle::Zero()}, // in this case, no inclination, so zero
        {Angle::radians(kPi)} // radial orbits are weird case here - will need a few extra tests
       }},
      {"parabolic",
       {OrbitalPhysics::Shape::parabolic,
        {std::nan("parabolic orbit - no semiMajor Axis")},
        1.0, // parabolic orbits are ecc = 1
        {std::numeric_limits<double>::infinity()},
        {0.4313007372},// subtly different meaning of "sweep" for parabolic orbits
        {Angle::degrees(3.1798301199)},
        {Angle::radians(kPi)},
        {Angle::degrees(71.5915198294)}
       }},
      {"hyperbolic",
        {OrbitalPhysics::Shape::hyperbolic,
         {-64.2068965517},
         hyperbolicEcc,
         {std::numeric_limits<double>::infinity()},
         {0.083872062},
        {Angle::degrees(7.6629523069)},
        {Angle::radians(-2.417342652841646)},
        {Angle::degrees(160.2543589611)}
       }},
      {"elliptical",
       {OrbitalPhysics::Shape::elliptical,
        {68.962962963},
        ellipticalEcc,
        {83.3899855854},
        {0.0753470008},
        {Angle::degrees(166.3442145528)},
        {Angle::radians(1.6738779353175968)},
        {Angle::degrees(11.2248489655)}
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

  EXPECT_NEAR(expectedSemiMajorAxis.m, actualSemiMajorAxis.m, 1e-10);
}

TEST_P(OrbitalPhysicsTest, eccentricity){
  const auto caseName = GetParam();
  const OrbitalPhysics& orbit = testCaseMap.at(caseName);
  const double expectedEccentricity = testExpectationsMap.at(caseName).eccentricity;

  // Since we're indirectly calculating eccentricity above, make sure they align
  // with our physical intuition:
  // elliptical orbits between 0 & 1, hyperbolic > 1
  // (parabolic expectation = 1 set explicitly above)
  if("hyperbolic" == caseName){
    EXPECT_GT(expectedEccentricity, 1.0);
  }
  else if("elliptical" == caseName){
    EXPECT_GT(expectedEccentricity, 0.0);
    EXPECT_LT(expectedEccentricity, 1.0);
  }

  EXPECT_EQ(expectedEccentricity, orbit.eccentricity());

  // the above calcs use the ecc vector to calculate eccentricity, but
  // we can calculate eccentricity a priori from the physical parameters
  const auto& params = paramsMap.at(GetParam());
  const auto& h = params.specificAngularMomentum().rawVector();
  const double hSquared = h.dot(h);
  const double num = 2 * params.specificEnergy().e * hSquared;
  const double muSquared = params.stdGravParam().mu * params.stdGravParam().mu;
  const double alternateEcc = sqrt(1 + num/(muSquared));

  EXPECT_EQ(alternateEcc, orbit.eccentricity());
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

TEST_P(OrbitalPhysicsTest, argumentOfPeriapsis){
  const OrbitalPhysics& orbit = testCaseMap.at(GetParam());
  const Angle expectedArgPeriapsis = testExpectationsMap.at(GetParam()).argumentOfPeriapsis;

  const auto actualArgPeriapsis = orbit.argumentOfPeriapsis();

  Angle angularDifference= expectedArgPeriapsis - actualArgPeriapsis;

  EXPECT_LT(std::fabs(angularDifference.getDegrees()), 0.1)
      << "expected " << expectedArgPeriapsis.getDegrees() << "° actual "
      << actualArgPeriapsis.getDegrees() << "°";

}

INSTANTIATE_TEST_SUITE_P(OrbitalPhysicsTest, OrbitalPhysicsTest,
                         Values("unitCircle", "freeFall", "parabolic", "hyperbolic", "elliptical"));

} // namespace
} // namespace orb_mech
