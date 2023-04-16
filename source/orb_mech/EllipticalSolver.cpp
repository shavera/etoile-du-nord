#include "EllipticalSolver.h"

namespace orb_mech {

namespace {
Angle meanAnomalyFromState(const OrbitalKernel& kernel);
}  // namespace

EllipticalSolver::EllipticalSolver(const OrbitalKernel& kernel)
    : AbstractSolver{kernel} {
  meanAnomalyAtEpoch_ = meanAnomalyFromState(kernel);
}

//void EllipticalSolver::updateState() {
//  meanAnomalyAtEpoch_ = meanAnomalyFromState(kernel_);
//}

// Angle EllipticalSolver::trueAnomalyAtTime(Seconds time) const{
//   return Angle::Zero();
// }

Angle EllipticalSolver::meanAnomalyAtEpoch() const {
  return Angle::Zero();
}

// AbstractSolver::VelocitySolver EllipticalSolver::velocitySolver() const {
//   return [](Angle, const ElementsGenerator&){return
//   VelocityInfo{MetersPerSecond{}, Angle::Zero()};};
// }

namespace {
Angle trueAnomalyFromState(const CartesianVector& eccentricityVector,
                           const StateVector& state) {
  return Angle::Zero();
}

Angle eccentricAnomalyFromTrueAnomaly(Angle trueAnomaly, double eccentricity) {
  return Angle::Zero();
}

Angle meanAnomalyFromEccentricAnomaly(Angle eccentricAnomaly,
                                      double eccentricity) {
  return Angle::Zero();
}

Angle meanAnomalyFromState(const OrbitalKernel& kernel) {
  const auto trueAnomaly =
      trueAnomalyFromState(kernel.eccentricityVector(), kernel.stateAtEpoch());
  const double eccentricity = kernel.eccentricityVector().norm();
  const auto eccAnomaly =
      eccentricAnomalyFromTrueAnomaly(trueAnomaly, eccentricity);
  const auto meanAnomaly =
      meanAnomalyFromEccentricAnomaly(eccAnomaly, eccentricity);
  return meanAnomaly;
}

}  // namespace

}  // namespace orb_mech
