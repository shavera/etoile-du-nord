#include "orb_mech/CartesianVector.h"

namespace orb_mech {

CartesianVector::CartesianVector(const double x, const double y, const double z)
  : vector_{x, y, z}
{}

double CartesianVector::x() const{
  return vector_.x();
}

double CartesianVector::y() const{
  return vector_.y();
}

double CartesianVector::z() const {
  return vector_.z();
}

double CartesianVector::norm() const {
  return vector_.norm();
}

CartesianVector CartesianVector::normalizedVector() const {
  CartesianVector normalizedVector;
  normalizedVector.vector_ = this->vector_.normalized();
  return normalizedVector;
}

double CartesianVector::dot(const CartesianVector& vector1, const CartesianVector& vector2) {
  return vector1.vector_.dot(vector2.vector_);
}

CartesianVector CartesianVector::cross(const CartesianVector& leftVector, const CartesianVector& rightVector) {
  Eigen::Vector3d result = leftVector.vector_.cross(rightVector.vector_);
  return CartesianVector{result};
}

double CartesianVector::dot(const CartesianVector& other) const {
  return CartesianVector::dot(*this, other);
}

CartesianVector CartesianVector::cross(const CartesianVector& rightVector) const {
  return CartesianVector::cross(*this, rightVector);
}

bool CartesianVector::operator==(const CartesianVector& other) const {
  return vector_ == other.vector_;
}

bool CartesianVector::operator!=(const CartesianVector &other) const {
  return !(*this == other);
}

CartesianVector CartesianVector::operator*(double scale) const {
  return CartesianVector{scale*vector_};
}

CartesianVector CartesianVector::operator+(const CartesianVector &other) const {
  return CartesianVector{vector_ + other.vector_};
}

CartesianVector CartesianVector::operator-(const CartesianVector &other) const {
  return CartesianVector{vector_ - other.vector_};
}

double CartesianVector::separation(const CartesianVector& other) const {
  return (vector_-other.vector_).norm();
}

} // namespace orb_mech