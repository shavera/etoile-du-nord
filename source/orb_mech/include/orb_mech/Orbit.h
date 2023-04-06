#pragma once

#include "Body.h"
#include "CartesianVector.h"
#include "units.h"

#include <memory>

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

/// Interface to the overall orbital mechanics problem for one specific object in orbit
class Orbit{
public:
  static std::unique_ptr<Orbit> makeOrbit();
  virtual ~Orbit() = default;

  virtual void applyDeltaV(const VelocityVector& deltaV, Seconds applicationTime) = 0;

  [[nodiscard]] virtual OrbitalElements orbitalElements() const = 0;

  [[nodiscard]] virtual StateVector stateAtTime(Seconds time) = 0;
};

} // namespace orb_mech
