#pragma once

#include <numbers>
#include <cmath>

/**
 * In the future, should adopt a proper units library. For now, just using
 * my own (very simple) strong typing so I can keep moving forward.
 */

struct Meters{
  double m;
};

struct Kilograms{
  double kg;
};

struct Seconds{
  double s;
};

struct MetersPerSecond{
  double mps;
};

struct StandardGravParam{
  double mu;
};

// nominally (m/s)^2 - energy of orbit without dealing with object mass
struct SpecificEnergy{
  double e;
};

// nominally m^2/s - angular momentum without dealing with object mass
struct SpecificAngularMomentum{
  double h;
};

// angular velocity in radians per second
struct RadiansPerSecond{
  double w;
};

// normalizes to ±Pi
class Angle{
public:
  static Angle degrees(double deg){
    return Angle(Angle::deg_to_rad(deg));
  }
  static Angle radians(double rad){
    return Angle(rad);
  }
  [[nodiscard]] double getDegrees() const{return rad_*180.0/std::numbers::pi;}
  [[nodiscard]] double getRadians() const{return rad_;}

  Angle& operator+(const Angle& other){
    rad_ = normalizeRadians(rad_ + other.rad_);
    return *this;
  }

  Angle& operator-(const Angle& other){
    rad_ = normalizeRadians(rad_ - other.rad_);
    return *this;
  }

private:
  Angle() = delete;

  static double normalizeRadians(double radians){
    return std::remainder(radians, 2*std::numbers::pi);
  }

  static double deg_to_rad(double deg){
    return deg*std::numbers::pi/180.0;
  }

  Angle(double rad)
      : rad_{ normalizeRadians(rad)}
  {}

  double rad_;
};

static Angle operator+(const Angle& a1, const Angle& a2){
  return Angle::radians(a1.getRadians() + a2.getRadians());
}

static Angle operator-(const Angle& a1, const Angle& a2){
  return Angle::radians(a1.getRadians() - a2.getRadians());
}
