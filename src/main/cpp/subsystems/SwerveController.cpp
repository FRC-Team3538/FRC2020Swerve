#include "subsystems/SwerveController.hpp"

SwerveController::SwerveController()
{
    ConfigureMotors();
}

/**
 * Main Function to control swerve drive
 * 
 * @param x double input for Left-Right on joystick
 * @param y double input for Forward-Backward on joystick
 * @param rotate double input for joystick rotate input
 * @param fieldCentric bool input to use field-centric control or robot-centric
 */
void SwerveController::SwerveDrive(double x, double y, double rotate, bool fieldCentric)
{
    double r = sqrt(pow(x, 2) + pow(y, 2));
    double theta = atan(y / x); // Radians

    // Snap to poles if within deadband
    theta = deadbandPoles(theta);

    // Scale the magnitude of the polar graph in order to preserve the angle
    double scaledR = pow(r, 3);

    x = scaledR * cos(theta);
    y = scaledR * sin(theta);

    // Convert to field-centric inputs if using field-centric control
    if(fieldCentric)
    {
        double temp = (y * cos(GetAngle())) + (x * sin(GetAngle()));
        x = (-y * sin(GetAngle())) + (x * cos(GetAngle()));
        y = temp;
    }

    // Calculate swerve commands based on inputs
    SK.Calculate(x, y, rotate);
    motorVals = SK.GetKinematics();

    // Send command to modules
    frontRight.setModule(motorVals[0][1], motorVals[0][0]);
    frontLeft.setModule(motorVals[1][1], motorVals[1][0]);
    backRight.setModule(motorVals[2][1], motorVals[2][0]);
    backLeft.setModule(motorVals[3][1], motorVals[3][0]);
}

/**
 * Returns robot angle heading
 * 
 * @return angle of robot -180 ~ 180
 */ 
double SwerveController::GetAngle()
{
    return pigeotto.GetFusedHeading();
}

/**
 * Tests for proximity to poles (0, pi/2, pi, 3pi/2)
 * and if within deadband, snaps it to the pole
 * 
 * @param input angle in radians
 * @return input angle, or pole if close enough
 */ 
double SwerveController::deadbandPoles(double input)
{
    double temp = input;
    input %= 360.0;

    if(abs(input - ((3* Constants::pi) / 2)) < poleDeadband)
    {
        return ((3* Constants::pi) / 2);
    }
    else if(abs(input - Constants::pi) < poleDeadband)
    {
        return Constants::pi;
    }
    else if(abs(input - (Constants::pi / 2)) < poleDeadband)
    {
        return (Constants::pi / 2);
    }
    else
    {
        return temp;
    }
    

}

void SwerveController::UpdateTelemetry()
{
}

void SwerveController::ConfigureMotors()
{
}