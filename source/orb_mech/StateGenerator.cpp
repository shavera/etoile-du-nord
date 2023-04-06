#include "StateGenerator.h"

#include <functional>

namespace orb_mech{

using namespace std::placeholders;

namespace {
// simple Newtonian solver; since we know derivative a priori, this is easy to use
double findFunctionRoot(const std::function<double(double)>& f,
                        const std::function<double(double)>& derivative,
                        double x0);
constexpr double kNewtonianTolerance{1e-6};
constexpr int kMaxIterations{100};

// the formula for eccentric anomaly arranged to = 0 for root finding
double eccentricAnomaly_f(double meanAnomaly, double eccentricity, double eccentricAnomaly);
// the derivative of eccentricAnomaly_f wrt eccentricAnomaly
double eccentricAnomaly_d(double eccentricity, double eccentricAnomaly);

} // namespace

StateVector generateStateAtEpoch(const OrbitalElements& elements){
  return {{{},{},{}}, {{}, {}, {}}};
}

Angle eccentricAnomaly(Angle meanAnomaly, double eccentricity){
  return Angle::radians(
      findFunctionRoot([=](double eccAnom){return eccentricAnomaly_f(meanAnomaly.getRadians(), eccentricity, eccAnom);},
                           [=](double eccAnom){return eccentricAnomaly_d(eccentricity, eccAnom);},
                           meanAnomaly.getRadians()));
}

namespace {

double findFunctionRoot(const std::function<double(double)>& f,
                        const std::function<double(double)>& derivative,
                        double x0) {
  double x{x0}, lastX, limit;
  int iteration = 0;
  do {
    lastX = x;
    const auto y = f(x);
    const auto dy = derivative(x);
    x = x - y/dy;
//    x = x - f(x) / derivative(x);
    limit = (0==lastX) ? kNewtonianTolerance : kNewtonianTolerance*lastX;
    if(++iteration > kMaxIterations){
      throw std::runtime_error{"Newtonian Solver could not find Root."};
    }
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
