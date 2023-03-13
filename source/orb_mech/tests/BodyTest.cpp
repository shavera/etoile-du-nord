#include "orb_mech/Body.h"

#include "gtest/gtest.h"

namespace orb_mech{
namespace {

TEST(BodyTest, bodyConstructorExceptions){
  {
    SCOPED_TRACE("negative mass");
    EXPECT_THROW(Body{-1.23}, std::domain_error);
  }
  {
    SCOPED_TRACE("zero mass");
    EXPECT_THROW(Body{0}, std::domain_error);
  }
}

TEST(BodyTest, stdGravParam){
  {
    SCOPED_TRACE("trivial 1 kg");
    Body body{1.0};
    EXPECT_NEAR(6.67430e-11, body.stdGravParam(), 1e-7*6.67430e-11);
  }
  {
    SCOPED_TRACE("100 kg");
    Body body{100.0};
    EXPECT_NEAR(6.67430e-09, body.stdGravParam(), 1e-7*6.67430e-09);
  }
  {
    SCOPED_TRACE("inverse G mass");
    float mass=1.0/6.67430e-11;
    Body body{mass};
    EXPECT_NEAR(1.0, body.stdGravParam(), 1e-7);
  }
  {
    SCOPED_TRACE("arbitrary mass");
    // using Earth Mass
    float mass=5.972168e24;
    Body body{mass};
    EXPECT_NEAR(3.986004418e14, body.stdGravParam(), 1e-7*3.986004418e14);
  }
}

TEST(DummyTest, Dummy){
  EXPECT_TRUE(false);
}

} // namespace
} // namespace orb_mech
