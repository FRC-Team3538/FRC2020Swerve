#include "subsystems/SwerveController.hpp"

SwerveController::SwerveController()
{
    ConfigureMotors();
    SetAngle(0.0);
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
    double theta = atan2(x, y); // Radians

    // Snap to poles if within deadband
    theta = deadbandPoles(theta);

    // Scale the magnitude of the polar graph in order to preserve the angle
    double scaledR = pow(r, 3);

    // Revert to cartesian
    x = scaledR * cos(theta);
    y = scaledR * sin(theta);

    // Convert to field-centric inputs if using field-centric control
    if (fieldCentric)
    {
        double robotAng = GetAngle() * (Constants::pi / 180.0); // Degrees to Radians
        double temp = (y * cos(robotAng)) + (x * sin(robotAng));
        x = (-y * sin(robotAng)) + (x * cos(robotAng));
        y = temp;
    }

    if ((x == 0.0) && (y == 0.0) && (rotate == 0.0))
    {
        double ang = Constants::frontRight.getAng();
        frontRight.setModule(-ang, 0.0);
        frontLeft.setModule(ang, 0.0);
        backRight.setModule(ang, 0.0);
        backLeft.setModule(-ang, 0.0);
    }
    else
    {
        // Calculate swerve commands based on inputs
        SK.Calculate(x, y, rotate);
        motorVals = SK.GetKinematics();

        // Send command to modules
        frontRight.setModule(motorVals[0][1], -motorVals[0][0]);
        frontLeft.setModule(motorVals[1][1], -motorVals[1][0]);
        backRight.setModule(motorVals[2][1], -motorVals[2][0]);
        backLeft.setModule(motorVals[3][1], -motorVals[3][0]);
    }
}

void SwerveController::SetIdleMode(rev::CANSparkMax::IdleMode idlemode)
{
    frontRight.SetIdleMode(idlemode);
    frontLeft.SetIdleMode(idlemode);
    backRight.SetIdleMode(idlemode);
    backLeft.SetIdleMode(idlemode);
}

void SwerveController::TestSteering(double x, double y)
{
    // frontLeft.testTurning(x);
    // return;

    if (x == 0.0 && y == 0.0)
    {
        if (prevTheta != -420.0)
        {
            frontRight.setModule(prevTheta, 0.0);
            frontLeft.setModule(prevTheta, 0.0);
            backRight.setModule(prevTheta, 0.0);
            backLeft.setModule(prevTheta, 0.0);
            return;
        }
        else
        {
            return;
        }
    }
    //double r = sqrt(pow(x, 2) + pow(y, 2));
    double r = 0.0;
    double theta = atan2(x, y); // Radians

    // Snap to poles if within deadband
    theta = deadbandPoles(theta);

    theta *= (180.0 / Constants::pi); //Convert to Degrees

    // Change to a -180 ~ 180 Scale
    theta -= 180;
    theta = theta > 180.0 ? theta - 360.0 : theta;
    prevTheta = theta;

    frontRight.setModule(theta, r);
    frontLeft.setModule(theta, r);
    backRight.setModule(theta, r);
    backLeft.setModule(theta, r);
}

/**
 * Returns robot angle heading
 * 
 * @return angle of robot -180 ~ 180
 */
double SwerveController::GetAngle()
{
    double ang = pidgeotto.GetFusedHeading();
    ang *= -1;
    ang = fmod(ang, 360.0);
    if (ang < 0.0)
        ang += 360.0;
    return ang;
}

/**
 * Sets current fused heading
 * 
 * @param angle double input to set fused heading to angle
 */
void SwerveController::SetAngle(double angle)
{
    pidgeotto.SetFusedHeading(angle);
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
    input = fmod(input, 360.0);

    if (abs(input - ((3 * Constants::pi) / 2)) < poleDeadband)
    {
        return ((3 * Constants::pi) / 2);
    }
    else if (abs(input - Constants::pi) < poleDeadband)
    {
        return Constants::pi;
    }
    else if (abs(input - (Constants::pi / 2)) < poleDeadband)
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
    frc::SmartDashboard::PutNumber("fr Ang", frontRight.getModuleAngle());
    frc::SmartDashboard::PutNumber("fl Ang", frontLeft.getModuleAngle());
    frc::SmartDashboard::PutNumber("br Ang", backRight.getModuleAngle());
    frc::SmartDashboard::PutNumber("bl Ang", backLeft.getModuleAngle());
    frc::SmartDashboard::PutNumber("Robit Ang", GetAngle());
    frontRight.UpdateTelemetry();
    frontLeft.UpdateTelemetry();
    backRight.UpdateTelemetry();
    backLeft.UpdateTelemetry();
}

void SwerveController::ConfigureMotors()
{
}