#include "../EllipticalSolver.h"

#include "gtest/gtest.h"

#include <Eigen/Geometry>

namespace orb_mech {
namespace {

using namespace ::testing;
/* - awaiting update of orbital kernel
/// @test want to test basic c'tor in isolation so rest of tests can start
/// with a constructed solution.
TEST(EllipticalSolverCtorTest, constructorBootstrap){
    // can just use a simple trivial case here, initial position unit-x
    // true anom should be = 0, mean anom = 0
    const CartesianVector eccentricityVector{0.25,0,0};
    const PositionVector initialPosition{{1}, {},{}};
    const Seconds initialEpochTime{1.234};
    EllipticalSolver trivialSolution{eccentricityVector, initialPosition,
initialEpochTime}; EXPECT_EQ(initialEpochTime,
trivialSolution.mostRecentEpoch()); EXPECT_EQ(Angle::Zero(),
trivialSolution.meanAnomalyAtEpoch());
}

/// @test just make sure epoch time is updated when we call update state
TEST(EllipticalSolverUpdateTest, updatesEpochTime){
    const CartesianVector eccentricityVector{0,0,0};
    const PositionVector initialPosition{{1}, {},{}};
    const Seconds initialEpochTime{1.234};
    EllipticalSolver solver{eccentricityVector, initialPosition,
initialEpochTime}; const Seconds newEpochTime{3.45};
    solver.updateStateAtEpoch(eccentricityVector, {{1},{},{}}, newEpochTime);
    EXPECT_EQ(newEpochTime, solver.mostRecentEpoch());
}

class EllipticalSolverTestBase : public Test{
public:
    void SetUp() override {
        // Fail all these tests if the constructor bootstrap fails, don't want
to proceed ASSERT_EQ(initialEpochTime, solver.mostRecentEpoch()) << "See
EllipticalSolverCtorTest"; ASSERT_EQ(Angle::Zero(), solver.meanAnomalyAtEpoch())
<< "See EllipticalSolverCtorTest";
    }

    const CartesianVector eccVec{0.25, 0, 0};
    const PositionVector initialPosition{{1}, {0},{0}};
    const Seconds initialEpochTime{1};
    EllipticalSolver solver{eccVec, initialPosition, initialEpochTime};
};

/////////////////////////////////////////////////
/// Invariants testing - While we shouldn't use the class in a way that invokes
these error cases, put them in place
/// to handle unexpected uses or edge conditions unforeseen
/////////////////////////////////////////////////

enum class InvariantsTestCases{zeroPosition, circularEcc, parabolicEcc,
hyperbolicEcc}; const std::map<InvariantsTestCases, std::string>
invariantsCaseNames{ {InvariantsTestCases::zeroPosition, "zeroPosition"},
        {InvariantsTestCases::circularEcc, "circularEcc"},
        {InvariantsTestCases::parabolicEcc, "parabolicEcc"},
        {InvariantsTestCases::hyperbolicEcc, "hyperbolicEcc"},
};

class EllipticalSolverInvariantsTest : public EllipticalSolverTestBase, public
WithParamInterface<InvariantsTestCases>{ public: const
std::map<InvariantsTestCases, CartesianVector> eccVectors{
        {InvariantsTestCases::zeroPosition, {0.25, 0, 0}},
        {InvariantsTestCases::circularEcc, {0, 0, 0}},
        {InvariantsTestCases::parabolicEcc, {1, 0, 0}},
        {InvariantsTestCases::hyperbolicEcc, {1.1, 0, 0}},
    };
};

TEST_P(EllipticalSolverInvariantsTest, constructorThrows){
    auto testCase = GetParam();
    PositionVector position = InvariantsTestCases::zeroPosition == testCase ?
PositionVector {{0},{0},{0}} : initialPosition; const auto& eccVec =
eccVectors.at(testCase); EXPECT_THROW((EllipticalSolver{eccVec, position,
initialEpochTime}), std::invalid_argument);
}

TEST_P(EllipticalSolverInvariantsTest, updateThrows){
    auto testCase = GetParam();
    PositionVector position = InvariantsTestCases::zeroPosition == testCase ?
PositionVector {{0},{0},{0}} : initialPosition; const auto& eccVec =
eccVectors.at(testCase); EXPECT_THROW((solver.updateStateAtEpoch(eccVec,
position, initialEpochTime)), std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(EllipticalSolverInvariants,
EllipticalSolverInvariantsTest, Values(InvariantsTestCases::zeroPosition,
InvariantsTestCases::circularEcc, InvariantsTestCases::parabolicEcc,
InvariantsTestCases::hyperbolicEcc),
                         [](const auto& info){ return
invariantsCaseNames.at(info.param);});

/////////////////////////////////////////////////
/// Mean Anomaly at Epoch testing
/////////////////////////////////////////////////

enum class CaseOrientation{planar, rotated};
enum class AnomalySign{positive, negative};
enum class AnomalyBase{
    zeroOrPi, ///< for "positive" case use zero, for "negative" case, Pi
    halfpi, ///< pi/2
    thirdpi ///< pi/3
};

using TestDatum = std::tuple<CaseOrientation, AnomalySign, AnomalyBase>;

Angle generateTrueAnomaly(AnomalySign sign, AnomalyBase base){
    if(AnomalyBase::zeroOrPi == base){
        // if sign positive, true anom at 0, if negative, at pi
        return AnomalySign::positive == sign ? Angle::Zero() :
Angle::degrees(180);
    }
    const Angle baseValue = AnomalyBase::halfpi == base ? Angle::degrees(90) :
Angle::degrees(60); const Angle trueAnomaly = AnomalySign::positive == sign ?
baseValue : Angle::Zero() - baseValue; return trueAnomaly;
}

PositionVector generatePosition(Angle trueAnomaly){
    return PositionVector{{cos(trueAnomaly.getRadians())},
{sin(trueAnomaly.getRadians())}, {0}};
}

// apply a somewhat arbitrary rotation to the vector
CartesianVector rotate(const CartesianVector& vector){
    // at time of writing test, haven't yet added in our rotation matrices to
our library, so just hand-rolling this
    // for the test
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd{0.25*M_PI, Eigen::Vector3d::UnitX()}
        *Eigen::AngleAxisd{M_PI/3.0, Eigen::Vector3d::UnitZ()}
        *Eigen::AngleAxisd{-0.125*M_PI, Eigen::Vector3d::UnitX()};
    Eigen::Vector3d rawVector{vector.x(), vector.y(), vector.z()};
    Eigen::Vector3d rotatedVector = m*rawVector;
    return CartesianVector{rotatedVector.x(), rotatedVector.y(),
rotatedVector.z()};
}

PositionVector rotate(const PositionVector& position){
    const auto rotatedRaw = rotate(position.rawVector());
    return PositionVector{rotatedRaw};
}

// table of true anom values:
// [-pi/2, -pi/3, 0, pi/3, pi/2, pi]
// table of ecc anom values (for ecc=0.25):
// [-1.5707963267948966, -1.4228148660461128,
0, 1.4228148660461128, 1.5707963267948966, pi]
// table of mean anom
// [-1.3207963267948966, -1.1755471885226927,
0, 1.1755471885226927, 1.3207963267948966, pi]
// function antisymmetric about 0, so can just store two values simply
Angle getExpectedMeanAnomaly(AnomalyBase base, AnomalySign sign){
    static const Angle meanAnomalyForPiHalf{Angle::radians(1.3207963267948966)};
    static const Angle
meanAnomalyForPiThird{Angle::radians(1.1755471885226927)};
    if(AnomalyBase::zeroOrPi == base){
        return AnomalySign::positive == sign ? Angle::Zero() :
Angle::degrees(180);
    }
    const Angle baseAngle = AnomalyBase::halfpi == base ? meanAnomalyForPiHalf :
meanAnomalyForPiThird; return AnomalySign::positive == sign ? baseAngle :
Angle::Zero() - baseAngle;
}

class EllipticalSolverTest : public EllipticalSolverTestBase, public
WithParamInterface<TestDatum>{ public: void SetUp() override{
    // Fail all these tests if the constructor bootstrap fails, don't want to
proceed EllipticalSolverTestBase::SetUp();

    const auto& [orientation, sign, base] = GetParam();

    const Angle trueAnomaly = generateTrueAnomaly(sign, base);
    const PositionVector position = generatePosition(trueAnomaly);
    testPosition = CaseOrientation::planar == orientation ? position :
rotate(position); testEccVec = CaseOrientation::planar == orientation ? eccVec :
rotate(eccVec);

    expectedMeanAnomaly = getExpectedMeanAnomaly(base, sign);
  }

  PositionVector testPosition{initialPosition};
  CartesianVector testEccVec{eccVec};
  Angle expectedMeanAnomaly{Angle::Zero()};
};

TEST_P(EllipticalSolverTest, meanAnomalyAtEpochFromConstruction){
  EllipticalSolver newSolver{testEccVec, testPosition, {3.45}};

  const auto meanAnomaly = newSolver.meanAnomalyAtEpoch();

  double diff = std::fabs((meanAnomaly-expectedMeanAnomaly).getRadians());
  EXPECT_LT(diff, 1e-9) << "exp: " << expectedMeanAnomaly << " act: " <<
meanAnomaly;
}

TEST_P(EllipticalSolverTest, meanAnomalyAtEpochFromUpdate){
    solver.updateStateAtEpoch(testEccVec, testPosition, {3.45});

    const auto meanAnomaly = solver.meanAnomalyAtEpoch();

    double diff = std::fabs((meanAnomaly-expectedMeanAnomaly).getRadians());
    EXPECT_LT(diff, 1e-9) << "exp: " << expectedMeanAnomaly << " act: " <<
meanAnomaly;
}

std::string printer(const TestParamInfo<EllipticalSolverTest::ParamType>& info){
    const auto& [orientation, sign, base] = info.param;
    std::string out{};
    out += CaseOrientation::planar == orientation ? "Plan" : "Rotd";
    if(base == AnomalyBase::zeroOrPi){
        out += AnomalySign::positive == sign ? "Z0" : "Pi";
        return out;
    }
    out += AnomalySign::positive == sign ? "Pos" : "Neg";
    switch (base) {
        case AnomalyBase::halfpi: out += "HfPi"; break;
        case AnomalyBase::thirdpi: out += "ThPi"; break;
        default: break;
    }
    return out;
}

INSTANTIATE_TEST_SUITE_P(
    EllipticalSolverMeanAnomaliesTest,
    EllipticalSolverTest,
    Combine(Values(CaseOrientation::planar, CaseOrientation::rotated),
            Values(AnomalySign::positive, AnomalySign::negative),
            Values(AnomalyBase::zeroOrPi, AnomalyBase::halfpi,
AnomalyBase::thirdpi)), &printer);
*/
}  // namespace
}  // namespace orb_mech
