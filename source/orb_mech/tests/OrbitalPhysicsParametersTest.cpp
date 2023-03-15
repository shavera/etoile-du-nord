#include "../OrbitalPhysicsParameters.h"

#include "gtest/gtest.h"

namespace orb_mech{
namespace {

TEST(SpecificEnergyTest, specificOrbitalEnergy){
  struct InputDatum{
    StandardGravParam stdGravParam;
    StateVector stateVector;
  };

  auto energyCalc = [](const InputDatum& input)->SpecificEnergy {
    return OrbitalPhysicsParameters::specificOrbitalEnergy(input.stdGravParam, input.stateVector);
  };

  {
    SCOPED_TRACE("trivial calculation");
    // only the magnitude of velocity and position are relevant. This test and others below
    // will shuffle the orientation of the different vectors to demonstrate invariance
    const InputDatum input{1,
                           StateVector{
                               PositionVector {{1}, {0}, {0}},
                               VelocityVector{{0}, {1}, {0}}
                           }};
    // 1^2/2 - 1/1 = -1/2
    const auto energy = energyCalc(input);
    EXPECT_EQ(-0.5, energy.e);
  }
  {
    SCOPED_TRACE("parabolic orbit");
    // arbitrary numbers are reasonable because calculations are unit invariant
    // parabolic orbits are the edge case where the object falls toward a body
    // with zero velocity "at infinity"
    // speed = 14, distance = 8
    const InputDatum input{784,
                           StateVector{
                               PositionVector {{0}, {0}, {8}},
                               VelocityVector{{0}, {0}, {14}}
                           }};
    // 14^2/2 - 784/8 = 0
    const auto energy = energyCalc(input);
  }
  {
    SCOPED_TRACE("hyperbolic orbit");
    // hyberbolic orbits, the energy is the excess over the parabolic (e=0) case
    // so positively valued energy, e.g. more speed than parabolic case
    // speed 15, distance 8 ; 2,10,11 form a pythagorean quadruple = 15
    const InputDatum input{784,
                           StateVector{
                               PositionVector {{0}, {8}, {0}},
                               VelocityVector{{2}, {10}, {11}}
                           }};
    // 15^2/2 - 784/8 = 14.5
    const auto energy = energyCalc(input);
    EXPECT_EQ(14.5, energy.e);
  }
  {
    SCOPED_TRACE("elliptical orbit");
    // elliptical orbits don't ever get to infinity, returning to body,
    // thus negative energy, e.g. less speed than parabolic case
    // speed 13, distance 8 ; 3, 4, 12 form a pythagorean quadruple = 13
    const InputDatum input{784,
                           StateVector{
                               PositionVector {{8}, {0}, {0}},
                               VelocityVector{{3}, {12}, {4}}
                           }};
    // 13^2/2 - 94/8 = -13.5
    const auto energy = energyCalc(input);
    EXPECT_EQ(-13.5, energy.e);
  }
  {
    SCOPED_TRACE("Exception thrown for zero distance");
    const InputDatum input{100,
                           StateVector{
                               PositionVector {{0}, {0}, {0}},
                               VelocityVector{{3}, {12}, {4}}
                           }};
    EXPECT_THROW(energyCalc(input), std::runtime_error);
  }
}

TEST(SpecificAngularMomentumTest, specificAngularMomentum){
  {
    SCOPED_TRACE("Trivial Zero case");
    const StateVector stateVector{
        PositionVector{{},{},{}},
        VelocityVector{{},{},{}}
    };
    const auto angMomVec = OrbitalPhysicsParameters::specificAngularMomentum(stateVector);
    const SpecAngMomVector expected{{0}, {0}, {0}};
    EXPECT_EQ(expected, angMomVec);
  }
  {
    SCOPED_TRACE("Unit vector parallel");
    const StateVector stateVector{
        PositionVector{{1},{},{}},
        VelocityVector{{1},{},{}}
    };
    const auto angMomVec = OrbitalPhysicsParameters::specificAngularMomentum(stateVector);
    const SpecAngMomVector expected{{0}, {0}, {0}};
    EXPECT_EQ(expected, angMomVec);
  }
  {
    SCOPED_TRACE("Unit vector perpendicular");
    const StateVector stateVector{
        PositionVector{{1},{},{}},
        VelocityVector{{},{1},{}}
    };
    // X x Y = Z
    const auto angMomVec = OrbitalPhysicsParameters::specificAngularMomentum(stateVector);
    const SpecAngMomVector expected{{0}, {0}, {1}};
    EXPECT_EQ(expected, angMomVec);
  }
  {
    SCOPED_TRACE("Unit vector negative perpendicular");
    const StateVector stateVector{
        PositionVector{{},{1},{}},
        VelocityVector{{1},{},{}}
    };
    // Y x X = -Z
    const auto angMomVec = OrbitalPhysicsParameters::specificAngularMomentum(stateVector);
    const SpecAngMomVector expected{{0}, {0}, {-1}};
    EXPECT_EQ(expected, angMomVec);
  }
  {
    SCOPED_TRACE("Nontrivial calculation");
    const StateVector stateVector{
        PositionVector{{1.23},{-3.21},{7.12}},
        VelocityVector{{3.99},{7.85},{-1.22}}
    };
    const auto angMomVec = OrbitalPhysicsParameters::specificAngularMomentum(stateVector);
    const SpecAngMomVector expected{{-51.9758}, {29.9094}, {22.4634}};
    EXPECT_NEAR(expected.x().h, angMomVec.x().h, 1e-4);
    EXPECT_NEAR(expected.y().h, angMomVec.y().h, 1e-4);
    EXPECT_NEAR(expected.z().h, angMomVec.z().h, 1e-4);
  }
}

TEST(EccentricityVectorTest, eccentricityVector){
  {
    SCOPED_TRACE("simple unit vectors case");
    // considering physically correct scenario: R is unit x, V is unit Y,
    // R x V = unit Z = ang. mom vec, h
    // V x h = - unit X
    // letting std. grav param, mu, = 0.5: Vxh/mu - unit R = - 2 unit X + unit X = unit X
    // have to let mu be nonzero else this is a circular orbit, which is too trivial to
    // be useful.
    const StandardGravParam stdGravParam{0.5};
    const StateVector stateVector{
        PositionVector{{1},{0},{0}},
        VelocityVector{{0},{1},{0}}
    };
    const SpecAngMomVector specAngMomVector{{0}, {0}, {1}};
    const CartesianVector expectedEccentricityVector{1,0,0};

    const auto eccVec = OrbitalPhysicsParameters::eccentricityVector(stdGravParam, stateVector, specAngMomVector);
  }
  {
    SCOPED_TRACE("nontrivial vector physical case");
    const StateVector stateVector{
        PositionVector {{1.23}, {-3.45}, {6.78}},
        VelocityVector {{2.34}, {4.56}, {-7.89}}
    };
    // assumes that angular momentum calculator is tested well above:
    const SpecAngMomVector angMomVector{OrbitalPhysicsParameters::specificAngularMomentum(stateVector)};
    const StandardGravParam stdGravParam{2.78};
    // the following was hand calculated (in python), so may need to be "close enough"
    const CartesianVector expectedEccentricityVector{94.85316256811534, -0.5780592474686674, 26.70603736002381};

    const CartesianVector eccVector{OrbitalPhysicsParameters::eccentricityVector(stdGravParam, stateVector, angMomVector)};

    EXPECT_NEAR(expectedEccentricityVector.x(), eccVector.x(), 1e-6);
    EXPECT_NEAR(expectedEccentricityVector.y(), eccVector.y(), 1e-6);
    EXPECT_NEAR(expectedEccentricityVector.z(), eccVector.z(), 1e-6);

    // since this is meant to be a physically realistic calculation,
    // we can use the alternate form that only uses the state vector itself
    // as a cross-check: (V^2/mu - 1/|R|)R - ((R . V)/mu) V
    const auto& rvec{stateVector.position.rawVector()};
    const auto& vvec{stateVector.velocity.rawVector()};
    const auto mu = stdGravParam.mu;
    const double vSquared = (std::pow(stateVector.speed.mps, 2));
    const double rCoeff = (vSquared/mu - 1.0/stateVector.distance.m);
    const double vCoeff = rvec.dot(vvec)/mu;
    const CartesianVector erVec = rCoeff*rvec;
    const CartesianVector evVec = vCoeff*vvec;
    const CartesianVector alternateExpectedEcc = erVec - evVec;

    EXPECT_LT(alternateExpectedEcc.separation(eccVector), 1e-6);
  }
  {
    SCOPED_TRACE("nontrivial vector non-physical case");
    // This method doesn't restrict you to calculating the actual eccentricity vector
    // it does allow for a caller to provide an angular momentum vector that is not matched
    // with the state vector. This should not ever be called in this way, but it is available
    // this test will confirm that the result is still Vxh/mu - (unit R)
    const StateVector stateVector{
        PositionVector {{10}, {0}, {0}},
        VelocityVector {{0}, {5}, {0}}
    };
    // real ang. momentum is +50 Z
    const SpecAngMomVector angMomVector{{20}, {0}, {0}};
    const StandardGravParam stdGravParam{10};
    // Vxh = - 100 Z -> Vxh/mu = -10 Z.
    // unit R is just unit X
    const CartesianVector exxpectedEccVec{-1, 0, -10};

    const CartesianVector actualEccVec{OrbitalPhysicsParameters::eccentricityVector(stdGravParam, stateVector, angMomVector)};
    EXPECT_EQ(actualEccVec, exxpectedEccVec);
  }


}

} // namespace
} // namespace orb_mech
