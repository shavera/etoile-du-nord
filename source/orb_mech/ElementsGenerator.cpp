#include "ElementsGenerator.h"

#include <cmath>
#include <numbers>

namespace orb_mech{
namespace{
constexpr double kPi = std::numbers::pi;

ElementsGenerator::Shape f_shape(const SpecificEnergy& energy){
  if(energy.e < 0) { return ElementsGenerator::Shape::elliptical; }
  if(energy.e > 0) { return ElementsGenerator::Shape::hyperbolic; }
  return ElementsGenerator::Shape::parabolic;
}

Meters f_semiMajorAxis(const OrbitalKernel & physParam,
                       ElementsGenerator::Shape shape){
  if(ElementsGenerator::Shape::parabolic == shape){
    return Meters{std::nan("parabolic orbit - no semimajoraxis")};
  }
  return {-physParam.stdGravParam().mu/(2.0*physParam.specificEnergy().e)};
}

Seconds f_period(StandardGravParam stdGravParam, const Meters& semiMajorAxis,
                 ElementsGenerator::Shape shape){
  if(ElementsGenerator::Shape::elliptical == shape){
    const auto aCubed = std::pow(semiMajorAxis.m, 3);
    return {2*kPi*sqrt(aCubed/stdGravParam.mu)};
  }
  return {std::numeric_limits<double>::infinity()};
}

RadiansPerSecond f_sweep(ElementsGenerator::Shape shape, StandardGravParam stdGravParam, Meters semiMajorAxis, const SpecAngMomVector& angularMomentum){
  if(ElementsGenerator::Shape::parabolic == shape){
    const double hSquared = angularMomentum.rawVector().dot(angularMomentum.rawVector());
    const double r_p = hSquared/(2*stdGravParam.mu);
    return {sqrt(stdGravParam.mu/(2*pow(r_p, 3)))};
  }
  const auto aCubedAbsVal = std::fabs(std::pow(semiMajorAxis.m, 3));
  return {sqrt(stdGravParam.mu/aCubedAbsVal)};
}

Angle f_inclination(const SpecAngMomVector& angularMomentum){
  return Angle::radians(std::acos(angularMomentum.z().h/angularMomentum.rawVector().norm()));
}

CartesianVector f_ascNodeVec(const SpecAngMomVector& angularMomentum){
  return {-angularMomentum.y().h,angularMomentum.x().h,0};
}

Angle f_longitudeAscNode(const CartesianVector& ascNodeVec){
  if(ascNodeVec.x() == 0 && ascNodeVec.y() == 0){
    return Angle::Zero();
  }
  return Angle::radians(std::atan2(ascNodeVec.y(), ascNodeVec.x()));
}

Angle f_argumentOfPeriapsis(const CartesianVector& ascNodeVec, const CartesianVector& eccVec){
  if(ascNodeVec.x() == 0 && ascNodeVec.y() == 0){
    return Angle::radians(std::atan2(eccVec.y(), eccVec.x()));
  }
  return Angle::radians(std::acos(ascNodeVec.normalizedVector().dot(eccVec.normalizedVector())));
}

double f_eccentricity(ElementsGenerator::Shape shape, const CartesianVector& eccVec){
  // Just to avoid rounding errors: if parabolic, return 1.0
  if(ElementsGenerator::Shape::parabolic == shape){
    return 1.0;
  }
  return eccVec.norm();
}

} // namespace

ElementsGenerator::ElementsGenerator(OrbitalKernel physicsParameters)
  : physicsParameters_{std::move(physicsParameters)}
  , cache_{physicsParameters_}
{}

ElementsGenerator::Cache::Cache(const OrbitalKernel & physParam)
  : shape{f_shape(physParam.specificEnergy())}
  , semiMajorAxis{f_semiMajorAxis(physParam, shape)}
  , eccentricity{f_eccentricity(shape, physParam.eccentricityVector())}
  , period{f_period(physParam.stdGravParam(), semiMajorAxis, shape)}
  , sweep{f_sweep(shape, physParam.stdGravParam(), semiMajorAxis, physParam.specificAngularMomentum())}
  , inclination{f_inclination(physParam.specificAngularMomentum())}
  , vectorOfAscendingNode{f_ascNodeVec(physParam.specificAngularMomentum())}
  , longitudeOfAscendingNode{f_longitudeAscNode(vectorOfAscendingNode)}
  , argumentOfPeriapsis{f_argumentOfPeriapsis(vectorOfAscendingNode, physParam.eccentricityVector())}
{}

} // namespace orb_mech
