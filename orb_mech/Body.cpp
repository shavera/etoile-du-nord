#include "orb_mech/Body.h"

namespace orb_mech {

Body::Body(float mass)
  : mass_{mass}
  , stdGravParam_{0}
{}

} // namespace orb_mech