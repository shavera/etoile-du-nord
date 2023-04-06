#pragma once

#include "orb_mech/CartesianVector.h"
#include "orb_mech/Orbit.h"
#include "orb_mech/units.h"
#include "ElementsGenerator.h"

namespace orb_mech {

/**
 * Interface for a family of solvers that find state as a function of time.
 *
 */
class AbstractSolver {
public:
  AbstractSolver(const ElementsGenerator& elementsGenerator, Seconds epochTime)
      : elementsGenerator_{elementsGenerator}, mostRecentEpoch_{epochTime} {}

  virtual ~AbstractSolver() = default;

  [[nodiscard]] virtual StateVector stateAtTime(Seconds time) const = 0;

protected:
  const ElementsGenerator& elementsGenerator_;
  Seconds mostRecentEpoch_;
};

} // namespace orb_mech
