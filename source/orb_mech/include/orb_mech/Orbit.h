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

  struct AuxiliaryValues{
    Meters periapsisDistance;
    Meters semiLatusRectum;
  } auxiliaryValues;
};



} // namespace orb_mech
