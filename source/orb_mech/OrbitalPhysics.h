#pragma once

#include "orb_mech/CartesianVector.h"
#include "orb_mech/units.h"

namespace orb_mech{

/**
 * Orbits are characterized by two physical values, the energy of the orbit
 * and the angular momentum. All the other parameters derive from these.
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

  OrbitalPhysics(double specificEnergy, CartesianVector specificAngularMomentum);


};

} // namespace orb_mech
