#include "orb_mech/Body.h"

#include <stdexcept>
#include <sstream>

namespace orb_mech {

namespace {
constexpr float kGravConstant{6.67430e-11}; // in N*m^2/kg^2 units
}

Body::Body(float mass)
  : mass_{mass}
  , stdGravParam_{kGravConstant*mass}
{
  if(mass <= 0){
    std::stringstream ss;
    ss << "Body provided with mass " << mass_ << "kg which is not positive";
    throw std::domain_error{ss.str()};
  }
}

} // namespace orb_mech