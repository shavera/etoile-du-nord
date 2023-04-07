#include "../ElementsGenerator.h"

#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

#include <cmath>
#include <numbers>

namespace orb_mech {
namespace {

using namespace ::testing;

const auto kPi = std::numbers::pi;

// Orbital cases fall along several orthogonal definitions
// going to use combinatorics to try to cover all cases
// Hand generating all this data is unreasonable. The plain "OrbitalPhysicsTest"
// test will cover some basic scenarios, then this test file will work backwards
// starting with orbital elements and generating the state vector at epoch.
// that way the logic is tested out in OrbitalPhysicsTest, and this test
// checks calculations at edge cases

/// which kind of conic section is the orbit, what's it's behaviour "at
/// infinity"
enum class Shape {
  circular,  ///< technically an elliptical orbit, but a special case with easy
             ///< maths
  elliptical,  ///< orbit returns to body cannot escape to infinity
  parabolic,   ///< orbit never returns, velocity asymptotically approaches zero
               ///< at infinity - not physically likely case
  hyperbolic   ///< orbit never returns, velocity never approaches zero
};

/// Is the orbit in the X-Y Plane?
enum class Plane {
  planar,   ///< orbit lies entirely in X-Y plane. No Z component of position or
            ///< velocity
  inclined  ///< orbit is inclined to X-Y plane
};

/// How complex are the variables? are we using trivial unit variables or
/// something more complex? goal here is to allow for trivial cases to be easily
/// computed to check functionality and nontrivial cases to check calculation
/// accuracy
enum class Complexity {
  trivial,  ///< Initial position and velocity are simple vectors, unit vectors
            ///< if possible
  nontrivial  ///< Initial position and velocity are nontrivial
};

/// Does the orbit have angular momentum, or is it a simple "free fall" (radial)
/// case Dropping an object from some height is a kind of orbit, just one
/// without angular momentum the maths may have some tricky spots here, so need
/// to check this edge case
enum class Arc {
  radial,  ///< Orbit is radial with no angular momentum
  curved   ///< Orbit has nonzero angular momentum
};

using TestTuple = std::tuple<Shape, Plane, Complexity, Arc>;

struct InputData {
  StandardGravParam mu{};
  StateVector state{{{}, {}, {}}, {{}, {}, {}}};
};

struct ExpectedOutput {
  ElementsGenerator::Shape shape{ElementsGenerator::Shape::elliptical};
  Meters semiMajorAxis{0};
  Seconds period{0};
  RadiansPerSecond sweep{0};
  Angle inclination{Angle::Zero()};
  Angle longitudeOfAscendingNode{Angle::Zero()};
  Angle argumentOfPeriapsis{Angle::Zero()};
  double eccentricity{0};
};

struct TestDatum {
  InputData input;
  ExpectedOutput output;
};

class OrbitalPhysicsTests : public TestWithParam<TestTuple> {};

// disabled until we get test data generation in place.
TEST_P(OrbitalPhysicsTests, DISABLED_dummy) {
  FAIL();
}

TestDatum testDatum(const TestTuple& testCase) {
  return {};
}

const std::map<Shape, std::string> shapeStrings{
    {Shape::circular, "Circular"},
    {Shape::elliptical, "Elliptical"},
    {Shape::parabolic, "Parabolic"},
    {Shape::hyperbolic, "Hyperbolic"}};

const std::map<Plane, std::string> planeStrings{{Plane::planar, "Planar"},
                                                {Plane::inclined, "Inclined"}};

const std::map<Complexity, std::string> complexityStrings{
    {Complexity::trivial, "Trivial"},
    {Complexity::nontrivial, "Nontrivial"}};

const std::map<Arc, std::string> arcStrings{{Arc::radial, "Radial"},
                                            {Arc::curved, "Curved"}};

std::string testMatrixPrinter(
    const TestParamInfo<OrbitalPhysicsTests::ParamType>& info) {
  const auto [shape, plane, complexity, arc] = info.param;
  return shapeStrings.at(shape) + planeStrings.at(plane) +
         complexityStrings.at(complexity) + arcStrings.at(arc);
}

INSTANTIATE_TEST_SUITE_P(OrbitMatrix,
                         OrbitalPhysicsTests,
                         Combine(Values(Shape::circular,
                                        Shape::elliptical,
                                        Shape::parabolic,
                                        Shape::hyperbolic),
                                 Values(Plane::planar, Plane::inclined),
                                 Values(Complexity::trivial,
                                        Complexity::nontrivial),
                                 Values(Arc::radial, Arc::curved)),
                         testMatrixPrinter);

}  // namespace
}  // namespace orb_mech
