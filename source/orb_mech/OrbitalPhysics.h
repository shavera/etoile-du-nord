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
 * @warning these equations nominally use mks units, and I haven't spent any time
 * on the conversion to other systems
 */
class OrbitalPhysics {
public:
  /**
   * Calculate the characteristic energy of the orbit given an object's speed
   * @param stdGravParam The standard gravitational parameter G*M of the parent body at the root of the orbit
   * @param speed the magnitude of the object's velocity in orbit around the body
   * @param distance the magnitude of the radial vector between the body and the object
   * @return specific orbital energy in
   */
  static SpecificEnergy specificOrbitalEnergy(StandardGravParam stdGravParam, MetersPerSecond speed, Meters distance);
  static SpecAngMomVector specificAngularMomentum(const PositionVector& position, const VelocityVector& velocity);
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
