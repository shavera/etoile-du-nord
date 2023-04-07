#pragma once

#include "orb_mech/units.h"

#include "Eigen/Dense"

#include <array>

namespace orb_mech {

class CartesianVector {
 public:
  CartesianVector(double x, double y, double z);
  CartesianVector(const CartesianVector& other) = default;
  CartesianVector(CartesianVector&& other) noexcept = default;
  CartesianVector& operator=(const CartesianVector& other) = default;
  CartesianVector& operator=(CartesianVector&& other) noexcept = default;
  ~CartesianVector() = default;

  [[nodiscard]] double x() const;
  [[nodiscard]] double y() const;
  [[nodiscard]] double z() const;

  [[nodiscard]] double norm() const;

  [[nodiscard]] CartesianVector normalizedVector() const;

  static double dot(const CartesianVector& v1, const CartesianVector& v2);
  static CartesianVector cross(const CartesianVector& v1,
                               const CartesianVector& v2);
  [[nodiscard]] double dot(const CartesianVector& other) const;
  [[nodiscard]] CartesianVector cross(const CartesianVector& other) const;

  /// Magnitude of vector of difference between this vector and other
  /// useful for checking if two vectors are nearly equal
  [[nodiscard]] double separation(const CartesianVector& other) const;

  bool operator==(const CartesianVector& other) const;
  bool operator!=(const CartesianVector& other) const;

  [[nodiscard]] CartesianVector operator*(double scale) const;

  [[nodiscard]] CartesianVector operator+(const CartesianVector& other) const;
  [[nodiscard]] CartesianVector operator-(const CartesianVector& other) const;

 private:
  explicit CartesianVector() : CartesianVector(Eigen::Vector3d{0, 0, 0}) {}

  explicit CartesianVector(Eigen::Vector3d vector)
      : vector_{std::move(vector)} {}

  Eigen::Vector3d vector_;
};

static CartesianVector operator*(double scale, const CartesianVector& vec) {
  return CartesianVector{vec.x() * scale, vec.y() * scale, vec.z() * scale};
}

static std::ostream& operator<<(std::ostream& os, const CartesianVector& vec) {
  os << "{" << vec.x() << ", " << vec.y() << ", " << vec.z() << "}";
  return os;
}

template <typename UnitType>
class VectorQuantity {
 public:
  VectorQuantity(UnitType x, UnitType y, UnitType z)
      : rawVector_{deconstruct(x), deconstruct(y), deconstruct(z)} {}

  explicit VectorQuantity(CartesianVector initialVector)
      : rawVector_{std::move(initialVector)} {}

  VectorQuantity(const VectorQuantity& other) = default;
  VectorQuantity(VectorQuantity&& other) noexcept = default;
  VectorQuantity& operator=(const VectorQuantity& other) = default;
  VectorQuantity& operator=(VectorQuantity&& other) noexcept = default;
  ~VectorQuantity() = default;

  [[nodiscard]] UnitType x() const { return UnitType{rawVector_.x()}; }
  [[nodiscard]] UnitType y() const { return UnitType{rawVector_.y()}; }
  [[nodiscard]] UnitType z() const { return UnitType{rawVector_.z()}; }

  [[nodiscard]] UnitType norm() const { return UnitType{rawVector_.norm()}; }

  [[nodiscard]] VectorQuantity<UnitType> normalizedVector() const {
    VectorQuantity<UnitType> vectorQuantity;
    vectorQuantity.rawVector_ = rawVector_.normalizedVector();
    return vectorQuantity;
  }

  bool operator==(const VectorQuantity<UnitType>& other) const {
    return rawVector_ == other.rawVector_;
  }

  bool operator!=(const VectorQuantity<UnitType>& other) const {
    return rawVector_ != other.rawVector_;
  }

  // removed dot and cross products since those have different units than the
  // type of the base vector.
  // to do maths with them without investing in a full units library, access
  // to the basic vector is provided
  [[nodiscard]] const CartesianVector& rawVector() const { return rawVector_; }

 private:
  CartesianVector rawVector_;

  explicit VectorQuantity() : VectorQuantity{{0}, {0}, {0}} {}

  double deconstruct(const UnitType& quantity) {
    const auto& [value] = quantity;
    return value;
  }
};

using PositionVector = VectorQuantity<Meters>;
using VelocityVector = VectorQuantity<MetersPerSecond>;
using SpecAngMomVector = VectorQuantity<SpecificAngularMomentum>;

struct StateVector {
  StateVector(PositionVector _position, VelocityVector _velocity)
      : position{std::move(_position)},
        velocity{std::move(_velocity)},
        distance{position.norm()},
        speed{velocity.norm()} {}

  const PositionVector position;
  const VelocityVector velocity;
  /// magnitude of position vector
  const Meters distance;
  /// magnitude of velocity vector
  const MetersPerSecond speed;
};

}  // namespace orb_mech
