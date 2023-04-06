#include "../StateGenerator.h"

#include "gtest/gtest.h"

namespace orb_mech{
namespace {

TEST(EccentricAnomalyTest, eccentricAnomaly){
// M = E-e*sin(E) solve for E -> very easy to do the reverse for testing
// cases: [circular: e=0, elliptic 0<e<1, parabolic e=1, hyperbolic e>1]
// + some variations to capture wraparound
  {
    SCOPED_TRACE("circular");
    // circular is trivial, mean and eccentric anomaly are equal
    const Angle meanAnomaly{Angle::radians(1.234)};
    const Angle eccAnomaly = eccentricAnomaly(meanAnomaly, 0);
    EXPECT_EQ(meanAnomaly, eccAnomaly);
  }
  {
    SCOPED_TRACE("elliptical");
    const double eccentricity = 0.25;
    const Angle meanAnomaly{Angle::radians(0.0926282354 )};
    const Angle eccAnomaly = eccentricAnomaly(meanAnomaly, eccentricity);
    const Angle expectedEccAnomaly{Angle::radians(0.1234)};
    const double diff = std::fabs((expectedEccAnomaly - eccAnomaly).getRadians());
    EXPECT_LT(diff, 1e-9) << "exp " <<expectedEccAnomaly << " act " << eccAnomaly ;
  }
//  {
//    SCOPED_TRACE("parabolic");
//    const double eccentricity = 1;
//    const Angle meanAnomaly{Angle::radians(0.95)};
//    const Angle eccAnomaly = eccentricAnomaly(meanAnomaly, eccentricity);
//    const Angle expectedEccAnomaly{Angle::radians(0.7466461238)};
//    const double diff = std::fabs((expectedEccAnomaly - eccAnomaly).getRadians());
//    EXPECT_LT(diff, 1e-9);
//  }
//  {
//    SCOPED_TRACE("hyperbolic");
//    const double eccentricity = 2.34;
//    const Angle meanAnomaly{Angle::radians(0.12)};
//    const Angle eccAnomaly = eccentricAnomaly(meanAnomaly, eccentricity);
//    const Angle expectedEccAnomaly{Angle::radians(-0.1601265651)};
//    const double diff = std::fabs((expectedEccAnomaly - eccAnomaly).getRadians());
//    EXPECT_LT(diff, 1e-9);
//  }
}

} // namespace
} // namespace orb_mech
