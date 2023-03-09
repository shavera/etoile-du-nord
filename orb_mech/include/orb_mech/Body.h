#pragma once

namespace orb_mech {

class Body {
public:
  /**
   * Create a body
   * @param mass_kg - mass of object in kilograms - must be >0
   *
   * @throws std::domain_error if mass_kg is less than 0
   */
  explicit Body(float mass_kg);

  /**
   * Get the standard gravitational parameter (G * M) for this body
   * Using mks system - N*m^2/kg units (alternatively m^3/s^2)
   *
   * @return standard gravitational parameter (m^3/s^2)
   */
  [[nodiscard]] float stdGravParam() const {return stdGravParam_;}

private:
  const float mass_{0};
  const float stdGravParam_{0};
};

} // namespace orb_mech
