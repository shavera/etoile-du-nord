#include "../OrbitalPhysics.h"

#include "gtest/gtest.h"

namespace orb_mech{
namespace {

TEST(SpecificEnergyTest, specificOrbitalEnergy){
  struct InputDatum{
    StandardGravParam stdGravParam;
    StateVector stateVector;
  };

  auto energyCalc = [](const InputDatum& input)->SpecificEnergy {
      return OrbitalPhysics::specificOrbitalEnergy(input.stdGravParam, input.stateVector);
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
    // 1^2/2 - 1/1 = 1/2
    const auto energy = energyCalc(input);
    EXPECT_EQ(0.5, energy.e);
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

} // namespace
} // namespace orb_mech
