#pragma once

namespace orb_mech {

class Body {
public:
  Body(float mass);

  float stdGravParam() const {return stdGravParam_;}

private:
  const float mass_{0};
  const float stdGravParam_{0};
};

} // namespace orb_mech
