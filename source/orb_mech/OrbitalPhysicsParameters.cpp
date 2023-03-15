#include "OrbitalPhysicsParameters.h"

namespace orb_mech {

SpecificEnergy OrbitalPhysicsParameters::specificOrbitalEnergy(StandardGravParam stdGravParam, const StateVector& stateVector) {
  if(stateVector.distance.m == 0.0){
    throw std::runtime_error{"Cannot calculate orbital energy at this location. If this issue persists, may need to implement alternate solution"};
  }
  return {stateVector.speed.mps*stateVector.speed.mps/2.0 - stdGravParam.mu/stateVector.distance.m};
}

SpecAngMomVector OrbitalPhysicsParameters::specificAngularMomentum(const StateVector& stateVector) {
  return SpecAngMomVector{stateVector.position.rawVector().cross(stateVector.velocity.rawVector())};
}

CartesianVector OrbitalPhysicsParameters::eccentricityVector(
    StandardGravParam stdGravParam,
    const StateVector& stateVector,
    const SpecAngMomVector& specAngMomVector) {
  const CartesianVector firstVec = (1.0/stdGravParam.mu)*(stateVector.velocity.rawVector().cross(specAngMomVector.rawVector()));
  return {firstVec - stateVector.position.rawVector().normalizedVector()};
}

} // namespace orb_mech