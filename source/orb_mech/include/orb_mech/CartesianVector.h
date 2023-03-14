#pragma once

#include "Eigen/Dense"

#include <array>

namespace orb_mech {

class CartesianVector {
public:
  CartesianVector(double x, double y, double z);
  CartesianVector(const CartesianVector& other) = default;
  CartesianVector(CartesianVector&& other) = default;
  CartesianVector& operator=(const CartesianVector& other) = default;
  CartesianVector& operator=(CartesianVector&& other) = default;
  ~CartesianVector() = default;

  [[nodiscard]] double x() const;
  [[nodiscard]] double y() const;
  [[nodiscard]] double z() const;

  [[nodiscard]] double norm() const;

  [[nodiscard]] CartesianVector normalizedVector() const;

  static double dot(const CartesianVector& v1, const CartesianVector& v2);
  static CartesianVector cross(const CartesianVector& v1, const CartesianVector& v2);
  [[nodiscard]] double dot(const CartesianVector& other) const;
  [[nodiscard]] CartesianVector cross(const CartesianVector& other) const;

private:
  explicit CartesianVector();
  Eigen::Vector3d vector_;
};

struct StateVector{
  CartesianVector position, velocity;
};

} // namespace orb_mech
