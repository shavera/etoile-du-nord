#pragma once

#include "units.h"

namespace orb_mech {

enum class OrbitShape {
  elliptical,  ///< Orbit is elliptical; energy < 0; a > 0; definite period
  parabolic,   ///< Orbit is parabolic (unlikely); energy = 0 precisely; no
               ///< period
  hyperbolic   ///< Orbit is hyperbolic; energy > 0; a < 0; no period
};

enum class RadialCase {
  radial,  ///< Orbit has no non-radial component of velocity; "free fall" orbit
  conic    ///< Orbit is a conic section
};

struct OrbitalElements {
  Meters semiMajorAxis;
  double eccentricity;
  Angle inclination, longitudeOfAscendingNode, argumentOfPeriapsis,
      meanAnomalyAtEpoch;

  struct AuxiliaryValues {
    Meters periapsisDistance;
    Meters semiLatusRectum;
    OrbitShape shape;
    RadialCase radialCase;
  } auxiliaryValues;
};

}  // namespace orb_mech
