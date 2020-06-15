#pragma once

#include <cmath>
#include "lib/Vector2d.hpp"

using namespace std;

namespace Constants
{

//Global Constants
const double pi = atan(1) * 4;

//Drivetrain Constants (Assuming Rectangular)
const double trackWidth = 24.0; //Inches
const double wheelBase = 24.0;  //Inches
const double diagonal = sqrt(pow(trackWidth, 2) + pow(wheelBase, 2));

const Vector2d frontRight{(trackWidth / 2), (wheelBase / 2)};
const Vector2d frontLeft{-(trackWidth / 2), (wheelBase / 2)};
const Vector2d backRight{(trackWidth / 2), -(wheelBase / 2)};
const Vector2d backLeft{-(trackWidth / 2), -(wheelBase / 2)};

} // namespace Constants