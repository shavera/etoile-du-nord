#include "../EllipticalSolver.h"

#include "gtest/gtest.h"

namespace orb_mech{
namespace {

using namespace ::testing;

class EllipticalSolverTest : public Test{
public:
  void SetUp() override{
    solver = std::make_unique<EllipticalSolver>(eccentricityVector, initialPosition, initialEpochTime);
  }

  const CartesianVector eccentricityVector{0,0,0};
  const PositionVector initialPosition{{}, {},{}};
  const Seconds initialEpochTime{13.24};
  std::unique_ptr<EllipticalSolver> solver{nullptr};
};

TEST_F(EllipticalSolverTest, mostRecentEpoch){
  EXPECT_EQ(initialEpochTime, solver->mostRecentEpoch());
}

} // namespace
} // namespace orb_mech
