#pragma once

#include "Body.h"
#include "CartesianVector.h"
#include "units.h"

namespace orb_mech{

struct OrbitalElements{
  Meter semiMajorAxis;
  double eccentricity;
  Angle inclination,
      longitudeOfAscendingNode,
      argumentOfPeriapsis,
      meanAnomalyAtEpoch;
};

class Orbit {
public:
    Orbit(const Body& rootBody, const StateVector& stateVector);
    Orbit(const Body& rootBody, const OrbitalElements& orbitalElements);

    [[nodiscard]] const OrbitalElements& orbitalElements();

private:
  const Body& rootBody_;
  OrbitalElements elements_;
};

} // namespace orb_mech
