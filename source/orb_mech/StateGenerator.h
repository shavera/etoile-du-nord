#pragma once

#include "orb_mech/CartesianVector.h"
#include "orb_mech/Orbit.h"

namespace orb_mech{

/// probably a throwaway function just to stand up the program basics
/// the valuable function, in the future, will be generating state at a non-zero time
StateVector generateStateAtEpoch(const OrbitalElements& elements);

} // namespace orb_mech
