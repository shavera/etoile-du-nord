#include "OrbitImpl.h"

namespace orb_mech {

std::unique_ptr<Orbit> Orbit::makeOrbit() {
  return std::make_unique<OrbitImpl>();
}

void OrbitImpl::applyDeltaV(const VelocityVector& deltaV,
                            Seconds applicationTime) {}

OrbitalElements OrbitImpl::orbitalElements() const {
  return {
      {},      {}, Angle::Zero(), Angle::Zero(), Angle::Zero(), Angle::Zero(),
      {{}, {}}};
}

StateVector OrbitImpl::stateAtTime(Seconds time) {
  return {{{}, {}, {}}, {{}, {}, {}}};
}

}  // namespace orb_mech
