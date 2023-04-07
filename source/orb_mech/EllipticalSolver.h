#pragma once

#include "AbstractSolver.h"

namespace orb_mech {

class EllipticalSolver : public AbstractSolver{
public:
  EllipticalSolver(const OrbitalKernel& kernel);

  void updateState() override;

  [[nodiscard]] Angle meanAnomalyAtEpoch() const override;

//  [[nodiscard]] Angle trueAnomalyAtTime(Seconds time) const override;
//
//  [[nodiscard]] VelocitySolver velocitySolver() const override;

private:
  Angle meanAnomalyAtEpoch_{Angle::Zero()};
};

} // namespace orb_mech
