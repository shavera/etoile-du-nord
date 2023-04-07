#include "EllipticalSolver.h"

namespace orb_mech {

namespace {
Angle meanAnomalyFromState(const CartesianVector& eccentricityVector, const PositionVector& position);
} // namespace

EllipticalSolver::EllipticalSolver(const CartesianVector& eccentricityVector,
                                   const PositionVector& positionAtEpoch,
                                   Seconds epochTime)
  : AbstractSolver{epochTime}
  , meanAnomalyAtEpoch_{meanAnomalyFromState(eccentricityVector, positionAtEpoch)}
{}

void EllipticalSolver::updateStateAtEpoch(const CartesianVector& eccentricityVector, const PositionVector& positionVector, Seconds epoch){
  mostRecentEpoch_ = epoch;
  meanAnomalyAtEpoch_ = meanAnomalyFromState(eccentricityVector, positionVector);
}

//Angle EllipticalSolver::trueAnomalyAtTime(Seconds time) const{
//  return Angle::Zero();
//}

Angle EllipticalSolver::meanAnomalyAtEpoch() const {
  return Angle::Zero();
}

//AbstractSolver::VelocitySolver EllipticalSolver::velocitySolver() const {
//  return [](Angle, const ElementsGenerator&){return VelocityInfo{MetersPerSecond{}, Angle::Zero()};};
//}

namespace {
Angle trueAnomalyFromState(const CartesianVector& eccentricityVector, const PositionVector& position){
  return Angle::Zero();
}

Angle eccentricAnomalyFromTrueAnomaly(Angle trueAnomaly, double eccentricity){
  return Angle::Zero();
}

Angle meanAnomalyFromEccentricAnomaly(Angle eccentricAnomaly, double eccentricity){
  return Angle::Zero();
}

Angle meanAnomalyFromState(const CartesianVector& eccentricityVector, const PositionVector& position){
  const auto trueAnomaly = trueAnomalyFromState(eccentricityVector, position);
  const double eccentricity = eccentricityVector.norm();
  const auto eccAnomaly = eccentricAnomalyFromTrueAnomaly(trueAnomaly, eccentricity);
  const auto meanAnomaly = meanAnomalyFromEccentricAnomaly(eccAnomaly, eccentricity);
  return meanAnomaly;
}

} // namespace

} // namespace orb_mech
