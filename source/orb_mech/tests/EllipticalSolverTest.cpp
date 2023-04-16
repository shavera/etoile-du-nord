#include "../EllipticalSolver.h"

#include "gtest/gtest.h"

#include <Eigen/Geometry>

namespace orb_mech {
namespace {

using namespace ::testing;

const StateVector simpleState{{{1}, {0}, {0}}, {{0}, {1}, {0}}};

/// @test want to test basic c'tor in isolation so rest of tests can start
/// with a constructed solution.
TEST(EllipticalSolverCtorTest, constructorBootstrap) {
  // can just use a simple trivial case here, initial position unit-x, initial
  // velocity unit-y, mu = 0.8 for ecc = 0.25 in x direction
  OrbitalKernel kernel{{0.8}, simpleState, {0}};
  EllipticalSolver solver{kernel};
  EXPECT_EQ(Angle::Zero(), solver.meanAnomalyAtEpoch());
}

class EllipticalSolverTestBase : public Test {
 public:
  void SetUp() override {
    // Fail all these tests if the constructor bootstrap fails, don't wantto
    // proceed
    ASSERT_EQ(Angle::Zero(), solver.meanAnomalyAtEpoch())
        << "See EllipticalSolverCtorTest";
  }
  const Seconds initialEpochTime{1};
  const StandardGravParam stdGravParam{0.8};
  OrbitalKernel defaultKernel{stdGravParam, simpleState, initialEpochTime};
  EllipticalSolver solver{defaultKernel};
};

/////////////////////////////////////////////////
/// Invariants testing - While we shouldn't use the class in a way that invokes
/// these error cases, put them in place
/// to handle unexpected uses or edge conditions unforeseen
/////////////////////////////////////////////////

// orbital kernel already guarantees position vec is nonzero, don't need that
// case
enum class InvariantsTestCases { circular, parabolic, hyperbolic, radial };
const std::map<InvariantsTestCases, std::string> invariantsCaseNames{
    {InvariantsTestCases::circular, "circular"},
    {InvariantsTestCases::parabolic, "parabolic"},
    {InvariantsTestCases::hyperbolic, "hyperbolic"},
    {InvariantsTestCases::radial, "radial"}};

class EllipticalSolverInvariantsTest
    : public EllipticalSolverTestBase,
      public WithParamInterface<InvariantsTestCases> {
 public:
  // keep same mu, position, just modify velocity vector appropriately.
  const std::map<InvariantsTestCases, VelocityVector> velocityVectors{
      {InvariantsTestCases::circular, {{}, {0.4 * sqrt(5)}, {}}},
      {InvariantsTestCases::parabolic, {{}, {2.0 * sqrt(0.4)}, {}}},
      {InvariantsTestCases::hyperbolic,
       {{}, {0.6 * sqrt(5)}, {}}},  // should be ecc=1.25
      {InvariantsTestCases::radial, {{}, {0}, {}}}};
};

TEST_P(EllipticalSolverInvariantsTest, constructorThrows) {
  auto testCase = GetParam();
  const StateVector testState{simpleState.position,
                              velocityVectors.at(testCase)};
  const OrbitalKernel kernel{stdGravParam, testState, initialEpochTime};
  EXPECT_THROW((EllipticalSolver{kernel}), std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(EllipticalSolverInvariants,
                         EllipticalSolverInvariantsTest,
                         Values(InvariantsTestCases::circular,
                                InvariantsTestCases::parabolic,
                                InvariantsTestCases::hyperbolic,
                                InvariantsTestCases::radial),
                         [](const auto& info) {
                           return invariantsCaseNames.at(info.param);
                         });

/////////////////////////////////////////////////
/// Mean Anomaly at Epoch testing
/////////////////////////////////////////////////

enum class CaseOrientation { planar, rotated };
enum class AnomalySign { positive, negative };
enum class AnomalyBase {
  zeroOrPi,       ///< for "positive" case use zero, for "negative" case, Pi
  halfpi,         ///< pi/2
  thirdpi,        ///< pi/3
  threeQuarterPi  ///< 3pi/4
};

using TestDatum = std::tuple<CaseOrientation, AnomalySign, AnomalyBase>;

Angle generateTrueAnomaly(AnomalySign sign, AnomalyBase base) {
  if (AnomalyBase::zeroOrPi == base) {
    // if sign positive, true anom at 0, if negative, at pi
    return AnomalySign::positive == sign ? Angle::Zero() : Angle::degrees(180);
  }
  const Angle baseValue =
      AnomalyBase::halfpi == base ? Angle::degrees(90) : Angle::degrees(60);
  const Angle trueAnomaly =
      AnomalySign::positive == sign ? baseValue : Angle::Zero() - baseValue;
  return trueAnomaly;
}

PositionVector generatePosition(Angle trueAnomaly) {
  return PositionVector{
      {cos(trueAnomaly.getRadians())}, {sin(trueAnomaly.getRadians())}, {0}};
}

// apply a somewhat arbitrary rotation to the vector
CartesianVector rotate(const CartesianVector& vector) {
  // at time of writing test, haven't yet added in our rotation matrices to our
  // library, so just hand-rolling this for the test
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd{0.25 * M_PI, Eigen::Vector3d::UnitX()} *
      Eigen::AngleAxisd{M_PI / 3.0, Eigen::Vector3d::UnitZ()} *
      Eigen::AngleAxisd{-0.125 * M_PI, Eigen::Vector3d::UnitX()};
  Eigen::Vector3d rawVector{vector.x(), vector.y(), vector.z()};
  Eigen::Vector3d rotatedVector = m * rawVector;
  return CartesianVector{rotatedVector.x(), rotatedVector.y(),
                         rotatedVector.z()};
}

PositionVector rotate(const PositionVector& position) {
  const auto rotatedRaw = rotate(position.rawVector());
  return PositionVector{rotatedRaw};
}

// table of true anom values:
// [-3pi/4, -pi/2, -pi/3, 0, pi/3, pi/2, 3pi/4, pi]
// table of ecc anom values (for ecc=0.25):
// [-1.8234765819369751, -1.5707963267948966, -1.4228148660461128,
// 0, 1.4228148660461128, 1.5707963267948966, 1.8234765819369751, pi] table of mean anom
// [-1.5814151227990116, -1.3207963267948966, -1.1755471885226927,
// 0, 1.1755471885226927, 1.3207963267948966, 1.5814151227990116, pi] function antisymmetric about
// 0, so can just store two values simply
Angle getExpectedMeanAnomaly(AnomalyBase base, AnomalySign sign) {
  static const Angle meanAnomalyForPiHalf{Angle::radians(1.3207963267948966)};
  static const Angle meanAnomalyForPiThird{Angle::radians(1.1755471885226927)};
  static const Angle meanAnomalyForThreeQuarterPi{Angle::radians(1.5814151227990116)};
  if (AnomalyBase::zeroOrPi == base) {
    return AnomalySign::positive == sign ? Angle::Zero() : Angle::degrees(180);
  }

  Angle baseAngle{Angle::Zero()};
  switch (base) {
    case AnomalyBase::halfpi: baseAngle = meanAnomalyForPiHalf; break;
    case AnomalyBase::thirdpi: baseAngle = meanAnomalyForPiThird; break;
    case AnomalyBase::threeQuarterPi: baseAngle = meanAnomalyForThreeQuarterPi; break;
    default: break;
  }
  return AnomalySign::positive == sign ? baseAngle : Angle::Zero() - baseAngle;
}

class EllipticalSolverTest : public EllipticalSolverTestBase,
                             public WithParamInterface<TestDatum> {
 public:
  void SetUp() override {
    // Fail all these tests if the constructor bootstrap fails, don't want to
    // proceed
    EllipticalSolverTestBase::SetUp();

    const auto& [orientation, sign, base] = GetParam();

    const Angle trueAnomaly = generateTrueAnomaly(sign, base);
    const PositionVector position = generatePosition(trueAnomaly);
    testPosition =
        CaseOrientation::planar == orientation ? position : rotate(position);
    testEccVec =
        CaseOrientation::planar == orientation ? eccVec : rotate(eccVec);

    expectedMeanAnomaly = getExpectedMeanAnomaly(base, sign);
  }

  PositionVector testPosition{initialPosition};
  CartesianVector testEccVec{eccVec};
  Angle expectedMeanAnomaly{Angle::Zero()};
};

TEST_P(EllipticalSolverTest, meanAnomalyAtEpochFromConstruction) {
  EllipticalSolver newSolver{testEccVec, testPosition, {3.45}};

  const auto meanAnomaly = newSolver.meanAnomalyAtEpoch();

  double diff = std::fabs((meanAnomaly - expectedMeanAnomaly).getRadians());
  EXPECT_LT(diff, 1e-9) << "exp: " << expectedMeanAnomaly
                        << " act: " << meanAnomaly;
}

std::string printer(
    const TestParamInfo<EllipticalSolverTest::ParamType>& info) {
  const auto& [orientation, sign, base] = info.param;
  std::string out{};
  out += CaseOrientation::planar == orientation ? "Plan" : "Rotd";
  if (base == AnomalyBase::zeroOrPi) {
    out += AnomalySign::positive == sign ? "Z0" : "Pi";
    return out;
  }
  out += AnomalySign::positive == sign ? "Pos" : "Neg";
  switch (base) {
    case AnomalyBase::halfpi:
      out += "HfPi";
      break;
    case AnomalyBase::thirdpi:
      out += "ThPi";
      break;
    default:
      break;
  }
  return out;
}

INSTANTIATE_TEST_SUITE_P(
    EllipticalSolverMeanAnomaliesTest,
    EllipticalSolverTest,
    Combine(Values(CaseOrientation::planar, CaseOrientation::rotated),
            Values(AnomalySign::positive, AnomalySign::negative),
            Values(AnomalyBase::zeroOrPi,
                   AnomalyBase::halfpi,
                   AnomalyBase::thirdpi)),
    &printer);

}  // namespace
}  // namespace orb_mech
