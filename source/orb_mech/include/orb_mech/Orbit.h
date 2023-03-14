#pragma once

namespace orb_mech{

struct OrbitalElements{
  double
      semiMajorAxis,
      eccentricity,
      inclination,
      longitudeOfAscendingNode,
      argumentOfPeriapsis,
      meanAnomalyAtEpoch;
};

class Orbit {
public:

};

} // namespace orb_mech
