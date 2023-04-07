#pragma once

#include "OrbitalKernel.h"
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


/**
 * @brief generate _most_ orbital elements from the physics parameters.
 *
 * The overall orbit shape and orientation may be determined from these parameters
 * but where to initialize the body in that orbit requires knowledge of the
 * state vector when the body is "at" t=0. The parent physics class will use
 * these values and additional solver tools to generate mean anomaly at epoch
 */
class ElementsGenerator {
public:
  explicit ElementsGenerator(const OrbitalKernel& kernel);
  virtual ~ElementsGenerator() = default;

  /**
   * What kind of conic section is the orbit? Useful to determine if you need
   * to check
   * @return
   */
  [[nodiscard]] OrbitShape shape() const{ return cache_.shape; }

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
   *
   * @return
   */
  [[nodiscard]] double eccentricity() const{return cache_.eccentricity;}

  /**
   * Period - the time, in seconds, that it takes to complete one orbit.
   *
   * Only can be defined for elliptical orbits, T=2*pi*sqrt((a^3)/mu)
   *
   * @note parabolic and hyperbolic orbits never "complete" so there's no definition
   * for period in such cases. In this case, however, the physical meaning is an
   * "infinite" amount of time, so we'll use the std::infinity return value here.
   *
   * @return period in seconds if elliptical orbit; seconds = NaN else
   */
  [[nodiscard]] Seconds period() const{ return cache_.period; }

  /**
   * The mean motion of the object in orbit; how fast it would go in an "equivalent" circular orbit
   *
   * - For elliptical orbits, this is straightforward: 2pi/period - it goes 1 revolution (2pi radians) in one period
   *   - if one didn't have period, this would look like sqrt(mu/a^3)
   *
   * - For hyperbolic orbits, a similar value can be calculated as sqrt(mu/(-a^3))
   *
   * - For parabolic orbits, one could bypass the eccentric anomaly approach altogether with a different formulation,
   * but that formulation can be written such that there's a term sqrt(mu/(2*r_p^3)) where r_p is the distance of
   * closest approach, found by taking angular momentum h: r_p=(h^2)/(2*mu) - this is the value that will be returned
   * until I can get working around the eccentric anomaly calculations and see what I need for parabolic orbits.
   */
  [[nodiscard]] RadiansPerSecond sweep() const{ return cache_.sweep; }

  /**
   * Angle that the orbital plane makes with the XY global coordinate plane
   *
   * for values over 90°, this corresponds to a retrograde orbit
   *
   * @todo
   * @note for radial orbits, those with no angular momentum, this value cannot be calculated
   * At least one of the angles won't be necessary since there's not a plane of the orbit, it might be
   * this one. For now, it will return NaN, but it may be safe to arbitrarily set it to zero
   * -- on second thought, I think it makes the most sense to define radial orbits by inclination and
   * longitude of ascending node. Arg. of periapsis is an angle in the plane of the orbit, and thus
   * the least sensible angle. Inclination and longAscNode give enough information to orient the orbit.
   *
   * @return angle between 0 and 180° (or pi radians)
   */
  [[nodiscard]] Angle inclination() const{ return cache_.inclination; }

  /**
   * Angle in fixed coordinates (corresponding to longitude on the parent body) at which an inclined
   * node rises above the XY plane.
   *
   * For planar orbits, this will be nominally set to zero, as it doesn't really matter where it is located.
   */
  [[nodiscard]] Angle longitudeOfAscendingNode() const{return cache_.longitudeOfAscendingNode;}

  /**
   * Angle from the ascending node to the periapsis of the orbit, in the plane of the orbit.
   *
   * There are two perfectly symmetrical orbits:
   * - Circular orbit - arbitrarily set this to zero
   * - Radial orbit - since the "orbit" is symmetrical either direction, the solution is degenerate,
   * a vector radially inward or outward both "point" to the "periapsis" if there was one.
   *   - Given the physical case an object will likely collide with the body, selecting the radially
   *   inward vector (when the orbit is first constructed) points toward the
   *   "closest approach" the object will make, so select that as the argument of periapsis.
   *
   * @warning the radial case is not properly implemented yet, the test case is too trivial to be useful.
   */
  [[nodiscard]] Angle argumentOfPeriapsis() const{ return cache_.argumentOfPeriapsis; }

  // Physics update stuff: I think the program should be event-driven with a tick cycle
  // do we need to maybe have a system of registering callbacks on physics updates?
  // or is it possible to just make sure we do our physics in the right order and it's irrelevant?

  [[nodiscard]] Meters periapsisDistance() const{return cache_.periapsisDistance;}

  [[nodiscard]] Meters semiLatusRectum() const{return cache_.semiLatusRectum;}

  void refreshCache(const OrbitalKernel& kernel){cache_ = Cache{kernel};}

private:
  // one way to preserve invariance is to calculate this cache on construction
  // then, when we introduce physics/time updates, we add a mechanism to lock
  // the cache to external access, do our updates, then unlock the cache.
  struct Cache{
    explicit Cache(const OrbitalKernel& kernel);
    OrbitShape shape;
    Meters semiMajorAxis;
    double eccentricity;
    double angMomSquared; // don't want to deal with units right now. This will have to do
    Seconds period;
    RadiansPerSecond sweep;
    Angle inclination;
    CartesianVector vectorOfAscendingNode;
    Angle longitudeOfAscendingNode;
    Angle argumentOfPeriapsis;
    Meters semiLatusRectum;
    Meters periapsisDistance;
  } cache_;
};

} // namespace orb_mech
