#include "StateGenerator.h"

namespace orb_mech{

StateVector generateStateAtEpoch(const OrbitalElements& elements){
  return {{{},{},{}}, {{}, {}, {}}};
}

} // namespace orb_mech
