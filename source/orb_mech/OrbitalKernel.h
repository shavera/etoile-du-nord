#pragma once

#include "orb_mech/CartesianVector.h"
#include "orb_mech/Orbit.h"
#include "orb_mech/units.h"

namespace orb_mech {
/**
   * @brief helper class to initialize the Orbital Physics calculations from
   * the standard grav param and state vector
   *
   * class not struct, because we need to encapsulate the parameters to ensure
   * operations performed are performed on all together (assuring class invariance)
 */
class OrbitalKernel {
public:
  OrbitalKernel(StandardGravParam standardGravParam, const StateVector& stateVector)
      : stdGravParam_{standardGravParam}
      , specificEnergy_{specificOrbitalEnergy(stdGravParam_, stateVector)}
      , specificAngularMomentum_{specificAngularMomentum(stateVector)}
      , eccentricityVector_{eccentricityVector(standardGravParam, stateVector, specificAngularMomentum_)}
  {}
  /**
   * @brief Calculate the specific orbital energy for an orbit
   *
   * Specific orbital energy is the energy of the object in orbit divided by its mass. Since we are
   * explicitly treating our simulation as only having a central mass and neglecting the object
   * mass terms, the specific orbital energy is constant for any object in an orbit.
   *
   * Consider the equation e=v^2/2 - mu/r = -0.5 *(mu^2/h^2)(1-e^2)=-mu/2a
   * where v is the velocity, r is the distance, mu is the std. grav. param,
   * h is specific angular momentum, a is the semi major axis. Each form of the
   * equation contains some mathematical singularity, but that need not correspond
   * to a physical one. e.g. imagine an object dropped from some height that can pass through
   * the center of mass of the body, as it passes through the center of mass, r=0. Throughout
   * the orbit, h=0 because there's no angular momentum to the orbit. But the orbit has a
   * semi-major axis 'a' which is the height from which it was dropped, which is nonzero.
   *
   * This helper function will likely only be used to initialize an orbit, and then future propagation
   * will simply use the orbital information to determine position, so the singularity shouldn't matter.
   *
   * Thus, given the extremely unlikely case where an orbit is attempted to be initialized "at" zero,
   * the function is designed to forbid such a case and throw an exception.
   *
   * @throws std::runtime_error if position vector in stateVector has zero magnitude
   *
   * @param stdGravParam The standard gravitational parameter G*M of the parent body at the root of the orbit
   * @param stateVector the position and velocity of an object occupying this orbit at a moment in time
   *
   * @return specific orbital energy
   */
  static SpecificEnergy specificOrbitalEnergy(StandardGravParam stdGravParam, const StateVector& stateVector);

  /**
   * @brief Calculate the specific orbital angular momentum for an orbit
   *
   * Specific orbital angular momentum is the angular momentum of the object in orbit divided by its mass. Since we are
   * explicitly treating our simulation as only having a central mass and neglecting the object
   * mass terms, the specific orbital angular momentum is constant for any object in an orbit.
   *
   * @param stateVector the position and velocity of an object occupying this orbit at a moment in time
   * @return Vector specifying the angular momentum of the orbit
   */
  static SpecAngMomVector specificAngularMomentum(const StateVector& stateVector);

  /**
   * @brief Calculate the eccentricity vector for an orbit
   *
   * The eccentricity vector is a vector pointing toward the periapsis from [apoapsis, center, focus] (all collinear in this case)
   * with magnitude equal to the eccentricity of the orbit. This vector is used to determine the orientation of the
   * orbit within its plane.
   *
   * @warning should only use the angular momentum vector calculated from the corresponding state vector.
   * It is possible to calculate this from state vector alone, but using the angular momentum vector minimizes
   * operations needed to calculate it.
   *
   * @param stdGravParam The standard gravitational parameter G*M of the parent body at the root of the orbit
   * @param stateVector the position and velocity of an object occupying this orbit at a moment in time
   * @param specAngMomVector the specific angular momentum vector (should be calculated above using the same position vector)
   * @return
   */
  static CartesianVector eccentricityVector(
      StandardGravParam stdGravParam,
      const StateVector& stateVector,
      const SpecAngMomVector& specAngMomVector);

  [[nodiscard]] StandardGravParam stdGravParam() const{return stdGravParam_;}

  [[nodiscard]] SpecificEnergy specificEnergy() const{return specificEnergy_;}

  [[nodiscard]] const SpecAngMomVector& specificAngularMomentum() const{return specificAngularMomentum_;}

  [[nodiscard]] const CartesianVector& eccentricityVector() const{return eccentricityVector_;}

private:
  StandardGravParam stdGravParam_;
  SpecificEnergy specificEnergy_;
  SpecAngMomVector specificAngularMomentum_;
  CartesianVector eccentricityVector_;
};


} // namespace orb_mech
