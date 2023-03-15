#pragma once

#include "orb_mech/CartesianVector.h"
#include "orb_mech/Orbit.h"
#include "orb_mech/units.h"

#include <memory>

namespace orb_mech{

/**
 * Orbits are characterized by 3 values, roughly:
 * specific orbital energy - sets an overall length scale (semimajor axis)
 * specific angular momentum - sets plane of orbit and width of orbit
 * eccentricity vector - orients periapsis within orbit plane
 *
 * When applying an orbital perturbation (delta V), it affects the above three values in the following way:
 *
 * - The change in overall magnitude of velocity changes the orbital energy
 *   - Alternatively: can formulate "work" by taking dot product of Force and displacement and then shifting energy by work done
 *   - (be sure to account for any radial distance change during the burn, as that will need accounted for)
 * - The prograde component of delta V
 *   - Adds to angular momentum by RxDV
 *   - Adds to ecc. vec by DVxH (in radial direction)
 * - The radial component of delta V
 *   - No change to angular momentum
 *   - Add to ecc vec by DVxH (in prograde direction)
 * - The normal component of delta V
 *   - Add RxDV to angular momentum (changes magnitude and direction of vector)
 *   - No change to ecc vec
 *
 * Thus, we can characterise orbital perturbations solely by changes to these physical fields without
 * having to time integrate a "force" view on the object.
 *
 * @note these equations are pretty much unit-invariant so long as the units are consistent between inputs.
 * StandardGravParam absorbs a lot of the specific unit details by its choice of gravitational constant,
 * but that is determined elsewhere when it's handed to this class.
 */
class OrbitalPhysics {
public:
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
   * @param stateVector the position and velocity of an object occupying this orbit at a moment in time
   * @param specAngMomVector
   * @return
   */
  static CartesianVector eccentricityVector(const StateVector& stateVector, const SpecAngMomVector& specAngMomVector);

  OrbitalPhysics(
      StandardGravParam stdGravParam,
      SpecificEnergy specificEnergy,
      SpecAngMomVector specificAngularMomentum,
      CartesianVector eccentricityVector);

  [[nodiscard]] const SpecificEnergy& specificEnergy() const {return energy_;}
  [[nodiscard]] const SpecAngMomVector& specificAngularMomentum() const{return angMomVector_;}

  [[nodiscard]] Meters semiMajorAxis() const;

  [[nodiscard]] Seconds period() const;
  [[nodiscard]] RadiansPerSecond sweep() const;

  [[nodiscard]] Angle inclination() const;

  [[nodiscard]] Angle longitudeOfAscendingNode() const;

  [[nodiscard]] Angle argumentOfPeriapsis() const;

  [[nodiscard]] double eccentricity() const;



  // Physics update stuff: I think the program should be event-driven with a tick cycle
  // do we need to maybe have a system of registering callbacks on physics updates?
  // or is it possible to just make sure we do our physics in the right order and it's irrelevant?

private:
  const StandardGravParam stdGravParam_;
  SpecificEnergy energy_;
  SpecAngMomVector angMomVector_;
  CartesianVector eccentricityVector_;
};

} // namespace orb_mech
