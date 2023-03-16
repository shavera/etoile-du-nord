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

  OrbitalPhysics(OrbitalPhysicsParameters physicsParameters);

  enum class Shape{elliptical, parabolic, hyperbolic};

  /**
   * What kind of conic section is the orbit? Useful to determine if you need
   * to check
   * @return
   */
  [[nodiscard]] Shape shape() const{ return cache_.shape; }

  /**
   * SemiMajor Axis - using convention where negative values for hyperbolic orbits
   *
   * @warning If parabolic orbit, a semiMajorAxis cannot be defined.
   * It's very unlikely to physically occur as small rounding errors will
   * probably produce a non-zero energy, but we should allow it as a physical thing.
   * Thus, we allow Meters to be defined with NaN to indicate that case. However,
   * cautious users will use the OrbitalPhysics::shape() method to check shape
   * and be aware if this is expected to be NaN
   *
   * @return semiMajorAxis in meters if not parabolic orbit; meters = NaN if parabolic
   */
  [[nodiscard]] Meters semiMajorAxis() const{ return cache_.semiMajorAxis; }

  /**
   * Period - the time, in seconds, that it takes to complete one orbit.
   *
   * @note parabolic and hyperbolic orbits never "complete" so there's no definition
   * for period in such cases. In this case, however, the physical meaning is an
   * "infinite" amount of time, so we'll use the std::infinity return value here.
   *
   * @return period in seconds if elliptical orbit; seconds = NaN else
   */
  [[nodiscard]] Seconds period() const{ return cache_.period; }
  [[nodiscard]] RadiansPerSecond sweep() const{ return cache_.sweep; }

  [[nodiscard]] Angle inclination() const{ return cache_.inclination; }

  [[nodiscard]] Angle longitudeOfAscendingNode() const{return cache_.longitudeOfAscendingNode;}

  [[nodiscard]] Angle argumentOfPeriapsis() const{return cache_.argumentOfPeriapsis;}

  [[nodiscard]] double eccentricity() const{return cache_.eccentricity;}

  // Physics update stuff: I think the program should be event-driven with a tick cycle
  // do we need to maybe have a system of registering callbacks on physics updates?
  // or is it possible to just make sure we do our physics in the right order and it's irrelevant?

private:
  OrbitalPhysicsParameters physicsParameters_;

  // one way to preserve invariance is to calculate this cache on construction
  // then, when we introduce physics/time updates, we add a mechanism to lock
  // the cache to external access, do our updates, then unlock the cache.
  struct Cache{
    explicit Cache(const OrbitalPhysicsParameters& physicsParameters);
    Shape shape;
    Meters semiMajorAxis;
    Seconds period;
    RadiansPerSecond sweep;
    Angle inclination{Angle::Zero()};
    Angle longitudeOfAscendingNode{Angle::Zero()};
    Angle argumentOfPeriapsis{Angle::Zero()};
    double eccentricity{0};
  } cache_;
};

} // namespace orb_mech
