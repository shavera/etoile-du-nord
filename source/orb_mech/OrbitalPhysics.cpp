#include "OrbitalPhysics.h"

#include <cmath>
#include <numbers>

namespace orb_mech{
namespace{
constexpr double kPi = std::numbers::pi;

OrbitalPhysics::Shape f_shape(const SpecificEnergy& energy){
  if(energy.e < 0) { return OrbitalPhysics::Shape::elliptical; }
  if(energy.e > 0) { return OrbitalPhysics::Shape::hyperbolic; }
  return OrbitalPhysics::Shape::parabolic;
}

Meters f_semiMajorAxis(const OrbitalPhysicsParameters& physParam, OrbitalPhysics::Shape shape){
  if(OrbitalPhysics::Shape::parabolic == shape){
    return Meters{std::nan("parabolic orbit - no semimajoraxis")};
  }
  return {-physParam.stdGravParam().mu/(2.0*physParam.specificEnergy().e)};
}

Seconds f_period(const StandardGravParam& stdGravParam, const Meters& semiMajorAxis, OrbitalPhysics::Shape shape){
  if(OrbitalPhysics::Shape::elliptical == shape){
    const auto aCubed = std::pow(semiMajorAxis.m, 3);
    return {2*kPi*sqrt(aCubed/stdGravParam.mu)};
  }
  return {std::numeric_limits<double>::infinity()};
}

} // namespace

OrbitalPhysics::OrbitalPhysics(OrbitalPhysicsParameters physicsParameters)
  : physicsParameters_{std::move(physicsParameters)}
  , cache_{physicsParameters_}
{}

OrbitalPhysics::Cache::Cache(const OrbitalPhysicsParameters& physParam)
  : shape{f_shape(physParam.specificEnergy())}
  , semiMajorAxis{f_semiMajorAxis(physParam, shape)}
  , period{f_period(physParam.stdGravParam(), semiMajorAxis, shape)}
{}

} // namespace orb_mech
