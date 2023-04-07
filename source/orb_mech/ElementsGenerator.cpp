#include "ElementsGenerator.h"

#include <cmath>
#include <numbers>

namespace orb_mech {
namespace {
constexpr double kPi = std::numbers::pi;

OrbitShape f_shape(const SpecificEnergy& energy) {
  if (energy.e < 0) {
    return OrbitShape::elliptical;
  }
  if (energy.e > 0) {
    return OrbitShape::hyperbolic;
  }
  return OrbitShape::parabolic;
}

Meters f_semiMajorAxis(const OrbitalKernel& physParam, OrbitShape shape) {
  if (OrbitShape::parabolic == shape) {
    return Meters{std::nan("parabolic orbit - no semimajoraxis")};
  }
  return {-physParam.stdGravParam().mu / (2.0 * physParam.specificEnergy().e)};
}

Seconds f_period(StandardGravParam stdGravParam,
                 const Meters& semiMajorAxis,
                 OrbitShape shape) {
  if (OrbitShape::elliptical == shape) {
    const auto aCubed = std::pow(semiMajorAxis.m, 3);
    return {2 * kPi * sqrt(aCubed / stdGravParam.mu)};
  }
  return {std::numeric_limits<double>::infinity()};
}

RadiansPerSecond f_sweepParabolic(double angMomSquared,
                                  StandardGravParam stdGravParam) {
  const double r_p = angMomSquared / (2 * stdGravParam.mu);
  return {sqrt(stdGravParam.mu / (2 * pow(r_p, 3)))};
}

RadiansPerSecond f_sweep(StandardGravParam stdGravParam, Meters semiMajorAxis) {
  const auto aCubedAbsVal = std::fabs(std::pow(semiMajorAxis.m, 3));
  return {sqrt(stdGravParam.mu / aCubedAbsVal)};
}

Angle f_inclination(const SpecAngMomVector& angularMomentum) {
  return Angle::radians(
      std::acos(angularMomentum.z().h / angularMomentum.rawVector().norm()));
}

CartesianVector f_ascNodeVec(const SpecAngMomVector& angularMomentum) {
  return {-angularMomentum.y().h, angularMomentum.x().h, 0};
}

Angle f_longitudeAscNode(const CartesianVector& ascNodeVec) {
  if (ascNodeVec.x() == 0 && ascNodeVec.y() == 0) {
    return Angle::Zero();
  }
  return Angle::radians(std::atan2(ascNodeVec.y(), ascNodeVec.x()));
}

Angle f_argumentOfPeriapsis(const CartesianVector& ascNodeVec,
                            const CartesianVector& eccVec) {
  if (ascNodeVec.x() == 0 && ascNodeVec.y() == 0) {
    return Angle::radians(std::atan2(eccVec.y(), eccVec.x()));
  }
  return Angle::radians(
      std::acos(ascNodeVec.normalizedVector().dot(eccVec.normalizedVector())));
}

double f_eccentricity(OrbitShape shape, const CartesianVector& eccVec) {
  // Just to avoid rounding errors: if parabolic, return 1.0
  if (OrbitShape::parabolic == shape) {
    return 1.0;
  }
  return eccVec.norm();
}

// for use in non-parabolic cases
Meters f_periapsisDistance(Meters semiMajorAxis, double eccentricity) {
  return {semiMajorAxis.m * (1 - eccentricity)};
}

// needed for parabolae
Meters f_periapsisDistanceParabolic(Meters semiLatusRectum) {
  return {semiLatusRectum.m / 2};
}

Meters f_semiLatusRectum(Meters semiMajorAxis, double eccentricity) {
  return {semiMajorAxis.m * (1 - eccentricity * eccentricity)};
}

Meters f_semiLatusRectumParabolic(double angularMomentumSquared,
                                  StandardGravParam stdGravParam) {
  return {angularMomentumSquared / stdGravParam.mu};
}

}  // namespace

ElementsGenerator::ElementsGenerator(const OrbitalKernel& kernel)
    : cache_{kernel} {}

ElementsGenerator::Cache::Cache(const OrbitalKernel& kernel)
    : shape{f_shape(kernel.specificEnergy())},
      semiMajorAxis{f_semiMajorAxis(kernel, shape)},
      eccentricity{f_eccentricity(shape, kernel.eccentricityVector())},
      angMomSquared{kernel.specificAngularMomentum().rawVector().dot(
          kernel.specificAngularMomentum().rawVector())},
      period{f_period(kernel.stdGravParam(), semiMajorAxis, shape)},
      sweep{OrbitShape::parabolic == shape
                ? f_sweepParabolic(angMomSquared, kernel.stdGravParam())
                : f_sweep(kernel.stdGravParam(), semiMajorAxis)},
      inclination{f_inclination(kernel.specificAngularMomentum())},
      vectorOfAscendingNode{f_ascNodeVec(kernel.specificAngularMomentum())},
      longitudeOfAscendingNode{f_longitudeAscNode(vectorOfAscendingNode)},
      argumentOfPeriapsis{f_argumentOfPeriapsis(vectorOfAscendingNode,
                                                kernel.eccentricityVector())},
      semiLatusRectum{
          OrbitShape::parabolic == shape
              ? f_semiLatusRectumParabolic(angMomSquared, kernel.stdGravParam())
              : f_semiLatusRectum(semiMajorAxis, eccentricity)},
      periapsisDistance{
          OrbitShape::parabolic == shape
              ? f_periapsisDistanceParabolic(semiLatusRectum)
              : f_periapsisDistance(semiMajorAxis, eccentricity)} {}

}  // namespace orb_mech
