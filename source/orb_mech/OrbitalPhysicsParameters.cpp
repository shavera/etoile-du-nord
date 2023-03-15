#include "OrbitalPhysicsParameters.h"

namespace orb_mech {

SpecificEnergy OrbitalPhysicsParameters::specificOrbitalEnergy(StandardGravParam stdGravParam, const StateVector& stateVector) {
  return {};
}

SpecAngMomVector OrbitalPhysicsParameters::specificAngularMomentum(const StateVector& stateVector) {
  return {{0}, {0}, {0}};
}

CartesianVector OrbitalPhysicsParameters::eccentricityVector(
    StandardGravParam stdGravParam,
    const StateVector& stateVector,
    const SpecAngMomVector& specAngMomVector) {
  return {0, 0, 0};
}

} // namespace orb_mech