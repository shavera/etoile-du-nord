#pragma once

#include "orb_mech/CartesianVector.h"
#include "orb_mech/OrbitalElements.h"
#include "orb_mech/units.h"

#include <functional>

namespace orb_mech {

class ElementsGenerator;

/**
 * Interface for a family of solvers that find state as a function of time.
 *
 */
class AbstractSolver {
public:
  explicit AbstractSolver(Seconds epoch) : mostRecentEpoch_{epoch} {}

  virtual ~AbstractSolver() = default;

  virtual void updateStateAtEpoch(const CartesianVector& eccentricityVector, const PositionVector & positionVector, Seconds epoch) = 0;

  [[nodiscard]] Seconds mostRecentEpoch() const {return mostRecentEpoch_;}

  [[nodiscard]] virtual Angle meanAnomalyAtEpoch() const = 0;

  [[nodiscard]] virtual Angle trueAnomalyAtTime(Seconds time) const = 0;

  struct VelocityInfo{
    MetersPerSecond speed;
    Angle angle;
  };
  using VelocitySolver = std::function<VelocityInfo(Angle trueAnomaly, const ElementsGenerator& elementsGenerator)>;
  [[nodiscard]] virtual VelocitySolver velocitySolver() const = 0;

protected:
  Seconds mostRecentEpoch_{0};
};

} // namespace orb_mech
