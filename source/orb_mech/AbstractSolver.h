#pragma once

#include "OrbitalKernel.h"
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
  explicit AbstractSolver(const OrbitalKernel& kernel) : kernel_{kernel} {}

  virtual ~AbstractSolver() = default;

  virtual void updateState() = 0;

  [[nodiscard]] virtual Angle meanAnomalyAtEpoch() const = 0;

  //  [[nodiscard]] virtual Angle trueAnomalyAtTime(Seconds time) const = 0;
  //
  //  struct VelocityInfo{
  //    MetersPerSecond speed;
  //    Angle angle;
  //  };
  //  using VelocitySolver = std::function<VelocityInfo(Angle trueAnomaly, const
  //  ElementsGenerator& elementsGenerator)>;
  //  [[nodiscard]] virtual VelocitySolver velocitySolver() const = 0;

 protected:
  const OrbitalKernel& kernel_;
};

}  // namespace orb_mech
