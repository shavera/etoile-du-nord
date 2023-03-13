#pragma once

#include "Eigen/Dense"

#include <array>

namespace orb_mech {

class CartesianVector {
public:
  CartesianVector(double x, double y, double z);

  [[nodiscard]] inline double x() const;
  [[nodiscard]] inline double y() const;
  [[nodiscard]] inline double z() const;

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

} // namespace orb_mech
