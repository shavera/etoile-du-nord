#include "OrbitalPhysics.h"

namespace orb_mech{

Meters OrbitalPhysics::semiMajorAxis() const{
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
