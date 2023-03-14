#pragma once

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

};

} // namespace orb_mech
