#include "StateGenerator.h"

#include <functional>

namespace orb_mech{

namespace {
// simple Newtonian solver; since we know derivative a priori, this is easy to use
double findFunctionRoot(const std::function<double(double)>& f,
                        const std::function<double(double)>& derivative,
                        double x0,
                        double tolerance = 1e-6);

// the formula for eccentric anomaly arranged to = 0 for root finding
double eccentricAnomaly_f(double meanAnomaly, double eccentricity, double eccentricAnomaly);
// the derivative of eccentricAnomaly_f wrt eccentricAnomaly
double eccentricAnomaly_d(double eccentricity, double eccentricAnomaly);

} // namespace

StateVector generateStateAtEpoch(const OrbitalElements& elements){
  return {{{},{},{}}, {{}, {}, {}}};
}

Angle eccentricAnomaly(Angle meanAnomaly, double eccentricity){
  return Angle::Zero();
}

namespace {

double findFunctionRoot(const std::function<double(double)>& f,
                        const std::function<double(double)>& derivative,
                        double x0,
                        double tolerance) {
  double x{x0}, lastX, limit;
  do {
    lastX = x;
    x = x - f(x) / derivative(x);
    limit = (0==lastX) ? tolerance : tolerance*lastX;
  } while(limit < std::fabs(lastX - x));
  return x;
}

double eccentricAnomaly_f(double meanAnomaly, double eccentricity, double eccentricAnomaly){
  return eccentricAnomaly - eccentricity*std::sin(eccentricAnomaly) - meanAnomaly;
}

double eccentricAnomaly_d(double eccentricity, double eccentricAnomaly){
  return 1 - eccentricity*std::cos(eccentricAnomaly);
}

} // namespace

} // namespace orb_mech
