#pragma once

#include "AbstractSolver.h"

namespace orb_mech {

class EllipticalSolver : public AbstractSolver{
public:
  EllipticalSolver(const ElementsGenerator& elementsGenerator,
                   Seconds epochTime,
                   const PositionVector& positionAtEpoch);

  [[nodiscard]] StateVector stateAtTime(Seconds time) const override;

private:
  Angle meanAnomalyAtEpoch_{Angle::Zero()};
};

} // namespace orb_mech
