#include "subsystems/SwerveController.hpp"

SwerveController::SwerveController()
{
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
    double theta = atan(y / x); //Radians

    //Scale the magnitude of the polar graph in order to preserve the angle
    double scaledR = pow(r, 3);

    x = scaledR * cos(theta);
    y = scaledR * sin(theta);

    //Convert to field-centric inputs if using field-centric control
    if(fieldCentric)
    {
        double temp = (y * cos(GetAngle())) + (x * sin(GetAngle()));
        x = (-y * sin(GetAngle())) + (x * cos(GetAngle()));
        y = temp;
    }

    //Calculate swerve commands based on inputs
    SK.Calculate(x, y, rotate);
    motorVals = SK.GetKinematics();

    //Find most efficient command to send to modules
    Controller();

    //Send command to modules
    frontRight.setModule(motorVals[0][0]);
    frontRight.setModuleAngle(motorVals[0][1]);
    frontLeft.setModule(motorVals[1][0]);
    frontLeft.setModuleAngle(motorVals[1][1]);
    backRight.setModule(motorVals[2][0]);
    backRight.setModuleAngle(motorVals[2][1]);
    backLeft.setModule(motorVals[3][0]);
    backLeft.setModuleAngle(motorVals[3][1]);
}

/**
 * This is a function to find the most
 * efficient command to send a module
 */ 
void SwerveController::Controller()
{
    double errFR = frontRight.getModuleAngle() - motorVals[0][1];
    double errFL = frontLeft.getModuleAngle() - motorVals[1][1];
    double errBR = backRight.getModuleAngle() - motorVals[2][1];
    double errBL = backLeft.getModuleAngle() - motorVals[3][1];

    if (errFR > 180.0)
        errFR -= 360.0;
    if (errFR < -180.0)
        errFR += 360.0;

    if (errFL > 180.0)
        errFL -= 360.0;
    if (errFL < -180.0)
        errFL += 360.0;

    if (errBR > 180.0)
        errBR -= 360.0;
    if (errBR < -180.0)
        errBR += 360.0;

    if (errBL > 180.0)
        errBL -= 360.0;
    if (errBL < -180.0)
        errBL += 360.0;

    /**
     * Ether's Swerve math only accounts for the module going forward
     * Thus, if the module must turn more than 90 deg to hit its setpoint,
     * We can reverse motor directionality and change the setpoint by 180 deg
     * To achieve a faster, more efficient rotation
     */
    if (abs(errFR) > 90.0)
    {
        motorVals[0][0] = -motorVals[0][0];

        if (motorVals[0][1] > 0.0)
        {
            motorVals[0][1] -= 180.0;
        }
        else
        {
            motorVals[0][1] += 180.0;
        }
    }

    if (abs(errFL) > 90.0)
    {
        motorVals[1][0] = -motorVals[1][0];

        if (motorVals[1][1] > 0.0)
        {
            motorVals[1][1] -= 180.0;
        }
        else
        {
            motorVals[1][1] += 180.0;
        }
    }

    if (abs(errBR) > 90.0)
    {
        motorVals[2][0] = -motorVals[2][0];

        if (motorVals[2][1] > 0.0)
        {
            motorVals[2][1] -= 180.0;
        }
        else
        {
            motorVals[2][1] += 180.0;
        }
    }

    if (abs(errBL) > 90.0)
    {
        motorVals[3][0] = -motorVals[3][0];

        if (motorVals[3][1] > 0.0)
        {
            motorVals[3][1] -= 180.0;
        }
        else
        {
            motorVals[3][1] += 180.0;
        }
    }
}

double SwerveController::GetAngle()
{
    return pigeotto.GetFusedHeading();
}

void SwerveController::UpdateTelemetry()
{
}

void SwerveController::ConfigureMotors()
{
}