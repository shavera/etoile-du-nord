#pragma once

#include "orb_mech/units.h"
#include "OrbitalPhysicsParameters.h"

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

  OrbitalPhysics(OrbitalPhysicsParameters physicsParameters)
      : physicsParameters_{std::move(physicsParameters)}
  {}

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
  OrbitalPhysicsParameters physicsParameters_;
};

} // namespace orb_mech
