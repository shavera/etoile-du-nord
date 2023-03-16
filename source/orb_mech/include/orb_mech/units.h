#pragma once

#include <cmath>
#include <numbers>
#include <ostream>

/**
 * In the future, should adopt a proper units library. For now, just using
 * my own (very simple) strong typing so I can keep moving forward.
 */

struct Meters{
  double m;
  auto operator<=>(const Meters&) const = default;
};

static std::ostream& operator<<(std::ostream& os, const Meters& meters){
  os << meters.m << " m";
  return os;
}

struct Kilograms{
  double kg;
  auto operator<=>(const Kilograms&) const = default;
};

static std::ostream& operator<<(std::ostream& os, const Kilograms& kilos){
  os << kilos.kg << " kg";
  return os;
}

struct Seconds{
  double s;
  auto operator<=>(const Seconds&) const = default;
};

static std::ostream& operator<<(std::ostream& os, const Seconds& seconds){
  os << seconds.s << " sec";
  return os;
}

struct MetersPerSecond{
  double mps;
  auto operator<=>(const MetersPerSecond&) const = default;
};

static std::ostream& operator<<(std::ostream& os, const MetersPerSecond& speed){
  os << speed.mps << " m/s";
  return os;
}

struct StandardGravParam{
  double mu;
  auto operator<=>(const StandardGravParam&) const = default;
};

static std::ostream& operator<<(std::ostream& os, const StandardGravParam& mu){
  os << mu.mu << " m^3/s^2";
  return os;
}

// nominally (m/s)^2 - energy of orbit without dealing with object mass
struct SpecificEnergy{
  double e;
  auto operator<=>(const SpecificEnergy&) const = default;
};

static std::ostream& operator<<(std::ostream& os, const SpecificEnergy& energy){
  os << energy.e << " J/kg";
  return os;
}

// nominally m^2/s - angular momentum without dealing with object mass
struct SpecificAngularMomentum{
  double h;
  auto operator<=>(const SpecificAngularMomentum&) const = default;
};

static std::ostream& operator<<(std::ostream& os, const SpecificAngularMomentum& h){
  os << h.h << " m^2/s";
  return os;
}

// angular velocity in radians per second
struct RadiansPerSecond{
  double w;
  auto operator<=>(const RadiansPerSecond&) const = default;
};

static std::ostream& operator<<(std::ostream& os, const RadiansPerSecond& w){
  os << w.w << " rad/s";
  return os;
}

// normalizes to ±Pi
class Angle{
public:
  Angle() = delete;
  Angle(const Angle& other) = default;
  Angle(Angle&& other) noexcept = default;
  Angle& operator=(const Angle& other) = default;
  Angle& operator=(Angle&& other) noexcept = default;
  ~Angle() = default;

  static Angle degrees(double deg){
    return Angle{Angle::deg_to_rad(deg)};
  }
  static Angle radians(double rad){
    return Angle{rad};
  }

  static Angle Zero(){
    return Angle{0.0};
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
  static double normalizeRadians(double radians){
    return std::remainder(radians, 2*std::numbers::pi);
  }

  static double deg_to_rad(double deg){
    return deg*std::numbers::pi/180.0;
  }

  explicit Angle(double rad)
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

static std::ostream& operator<<(std::ostream& os, const Angle& angle){
  os << "[" << angle.getRadians() <<" rad, " << angle.getDegrees() << " deg]";
  return os;
}
