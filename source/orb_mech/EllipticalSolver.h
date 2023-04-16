#pragma once

#include "AbstractSolver.h"

namespace orb_mech {
/************************
 TODO for when I next pick this up (added 07 Apr 2023)
 - implement mean anom at epoch for elliptical solver
 - create a circular solver for circular cases (inclined v. not)
 - get a real units library in here. Mine is garbage.
 - Start fleshing out OrbitImpl class to be smart about how this is all
 being built and connected together
 - add True Anom -> state utility
   - this should get us to being able to do basic "loopback" testing of initial state
     - generate orbital parameters, request state vector at initial state, should be equivalent
 - add motion solvers to circular and elliptical orbit cases
   - including velocity solvers
 - integrate motion solution to produce state as a function of time for fixed orbits
   - circular and elliptical only at this time
 ----
 - create Python API to library
 - sketch/experiment with Python to validate basic data alignment
 - create systematic Python test framework for Orbit stuff
 - start filling out the test matrix for physical cases for circular and elliptical orbits
 - maybe we can bootstrap longer tests out of shorter ones. If we can show that short time
 segments are reasonably well aligned, then maybe we can assemble the golden image data out
 of orbit sectors where we correct for the initial state every, 10 seconds or so, to
 minimize rounding drift

*********************/
class EllipticalSolver : public AbstractSolver {
 public:
  explicit EllipticalSolver(const OrbitalKernel& kernel);

  [[nodiscard]] Angle meanAnomalyAtEpoch() const override;

  //  [[nodiscard]] Angle trueAnomalyAtTime(Seconds time) const override;
  //
  //  [[nodiscard]] VelocitySolver velocitySolver() const override;

 private:
  Angle meanAnomalyAtEpoch_{Angle::Zero()};
};

}  // namespace orb_mech
