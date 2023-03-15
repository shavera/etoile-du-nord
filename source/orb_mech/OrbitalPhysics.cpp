#include "OrbitalPhysics.h"

namespace orb_mech{

SpecificEnergy OrbitalPhysics::specificOrbitalEnergy(StandardGravParam stdGravParam, MetersPerSecond speed, Meters distance) {
  return {};
}

SpecAngMomVector OrbitalPhysics::specificAngularMomentum(const PositionVector& position, const VelocityVector& velocity) {
  return {{0}, {0}, {0}};
}

CartesianVector OrbitalPhysics::eccentricityVector(
    const StateVector &stateVector,
    const SpecAngMomVector &specAngMomVector) {
  return CartesianVector(0, 0, 0);
}

OrbitalPhysics::OrbitalPhysics(
    StandardGravParam stdGravParam,
    SpecificEnergy specificEnergy,
    SpecAngMomVector specificAngularMomentum,
    CartesianVector eccentricityVector)
  : stdGravParam_{stdGravParam}
  , energy_{specificEnergy}
  , angMomVector_{std::move(specificAngularMomentum)}
  , eccentricityVector_{std::move(eccentricityVector)}
{}

OrbitalElements OrbitalPhysics::semiMajorAxis() const{
  return {};
}

Seconds OrbitalPhysics::period() const{
  return {};
}

RadiansPerSecond OrbitalPhysics::sweep() const{
  return {};
}

Angle OrbitalPhysics::inclination() const {
  return Angle::radians(0);
}

Angle OrbitalPhysics::longitudeOfAscendingNode() const {
  return Angle::radians(0);
}

Angle OrbitalPhysics::argumentOfPeriapsis() const {
  return Angle::radians(0);
}

double OrbitalPhysics::eccentricity() const {
  return 0;
}

} // namespace orb_mech
