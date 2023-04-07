#pragma once

#include "orb_mech/Orbit.h"

namespace orb_mech {

class OrbitImpl final : public Orbit {
 public:
  void applyDeltaV(const VelocityVector& deltaV,
                   Seconds applicationTime) override;

  [[nodiscard]] OrbitalElements orbitalElements() const override;

  [[nodiscard]] StateVector stateAtTime(Seconds time) override;
};

}  // namespace orb_mech
