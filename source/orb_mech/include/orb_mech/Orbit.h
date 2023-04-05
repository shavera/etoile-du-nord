#pragma once

#include "Body.h"
#include "CartesianVector.h"
#include "units.h"

namespace orb_mech{

struct OrbitalElements{
  Meters semiMajorAxis;
  double eccentricity;
  Angle inclination,
      longitudeOfAscendingNode,
      argumentOfPeriapsis,
      meanAnomalyAtEpoch;
};

} // namespace orb_mech
