#include "EllipticalSolver.h"

namespace orb_mech {

namespace {
Angle trueAnomalyFromState(const CartesianVector& eccentricityVector, const PositionVector& position);

Angle eccentricAnomalyFromTrueAnomaly(Angle trueAnomaly, double eccentricity);

Angle meanAnomalyFromEccentricAnomaly(Angle eccentricAnomaly, double eccentricity);

Angle meanAnomalyFromState(const CartesianVector& eccentricityVector, const PositionVector& position);
} // namespace

EllipticalSolver::EllipticalSolver(const ElementsGenerator& elementsGenerator,
                                   Seconds epochTime,
                                   const PositionVector& positionAtEpoch)
  : AbstractSolver{elementsGenerator, epochTime}
//  , meanAnomalyAtEpoch_{meanAnomalyFromState(elementsGenerator.kernel().eccentricityVector(), positionAtEpoch)}
  , meanAnomalyAtEpoch_{Angle::Zero()}
{}

StateVector EllipticalSolver::stateAtTime(Seconds time) const{
  return StateVector{{{},{},{}},{{},{},{}}};
}

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
