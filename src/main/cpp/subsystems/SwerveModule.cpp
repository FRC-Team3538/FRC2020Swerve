#include "subsystems/SwerveModule.hpp"

/**
 * Constructor for a Swerve Module
 * 
 * @param moduleID char input for an ID of the module
 * @param rotID int input for CAN ID of rotation motor
 * @param driveID int input for CAN ID of driving motor
 * @param position Vected2d input for position of module relative to center of rotation (Assuming equal weight distribution)
 * @param encOffs double input for offset on zero for absolute encoder
 */
SwerveModule::SwerveModule(char moduleID, int rotID, int driveID, const Vector2d &position, double encOffset)
    : moduleID(moduleID), position(position), driveMotor(LazyTalonFX(driveID)), rotationMotor(LazySparkMax(rotID)), encOffset(encOffset)
{
    ConfigureMotors();
}

/**
 * Gets Module Angle
 *  
 * @return angle of module, relative to front of robot.
 */
double SwerveModule::getModuleAngle()
{
    return 0.0;
}

/**
 * Sets Module Power Output from a scale of -1~1
 * 
 * @param angle is the module's target angle -180 ~ 180
 * @param power is the module's power output
 */
void SwerveModule::setModule(double angle, double power)
{
    double err = getModuleAngle() - angle;

    /**
     * This is to account for the gap between -180 and 180
     */ 
    if (err > 180.0)
        err -= 360.0;
    if (err < -180.0)
        err += 360.0;

    /**
     * Ether's Swerve math only accounts for the module going forward
     * Thus, if the module must turn more than 90 deg to hit its setpoint,
     * We can reverse motor directionality and change the setpoint by 180 deg
     * To achieve a faster, more efficient rotation
     */
    if (abs(err) > 90.0)
    {
        power = -power;

        if (angle > 0.0)
        {
            angle -= 180.0;
        }
        else
        {
            angle += 180.0;
        }
    }

    driveMotor._Set(ControlMode::PercentOutput, power);
}

/**
 * Sets module's angle, relative to robot and relative to current position
 * 
 * @param target double input for module relative angle target
 */ 
void SwerveModule::setModuleAngleRel(double target)
{
    double d_error = target - prevErr;
    prevErr = target;

    if(abs(target) < kIz)
    {
        iAcc += target;
    }
    else
    {
        iAcc = 0;
    }

    double command = (kP * target) + (kI * iAcc) + (kD * d_error);
    rotationMotor._Set(command);
}

/**
 * Returns if module is at the targeted position
 * 
 * @return if the module is at targeted position
 */
bool SwerveModule::angleOnTarget()
{
    return false;
}

void SwerveModule::ConfigureMotors()
{
}

void SwerveModule::UpdateTelemetry()
{
}