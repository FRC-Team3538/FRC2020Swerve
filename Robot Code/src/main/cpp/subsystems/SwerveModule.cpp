#include "subsystems/SwerveModule.hpp"

/**
 * Constructor for a Swerve Module
 * 
 * @param moduleID char input for an ID of the module
 * @param rotID int input for CAN ID of rotation motor
 * @param driveID int input for CAN ID of driving motor
 * @param encID int input for DIO ID of absolute encoder
 * @param position Vected2d input for position of module relative to center of rotation (Assuming equal weight distribution)
 * @param encOffs double input for offset on zero for absolute encoder
 */
SwerveModule::SwerveModule(string moduleID, int rotID, int driveID, int encID, const Vector2d &position, double encOffset)
    : moduleID(moduleID), position(position), driveMotor(LazyTalonFX(driveID)), rotationMotor(LazySparkMax(rotID)), rotEnc(AnalogInput(encID)), encOffset(encOffset)
{
    rotEnc.SetAverageBits(2);
    ConfigureMotors();
}

/**
 * Gets Module Angle
 *  
 * @return angle of module, relative to front of robot.
 */
double SwerveModule::getModuleAngle()
{
    double ang = rotEnc.GetAverageVoltage();
    ang *= -rotScaleFac;
    ang += encOffset;
    ang = ang < -180.0 ? ang + 360 : ang;
    return ang; /*rotEnc.GetDistance()*/
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
     * This Swerve math only accounts for the module going forward
     * Thus, if the module must turn more than 90 deg to hit its setpoint,
     * We can reverse motor directionality and change the setpoint by 180 deg
     * To achieve a faster, more efficient rotation
     */
    if (abs(err) > 90.0)
    {
        power = -power;

        if (err > 0.0)
        {
            err -= 180.0;
        }
        else
        {
            err += 180.0;
        }
    }

    driveMotor._Set(ControlMode::PercentOutput, power);

    angleTarget += err;
    // if (angleOnTarget())
    // {
    //     rotationMotor._Set(0.0);
    //     return;
    // }
    frc::SmartDashboard::PutNumber("Err", err);
    setModuleAngleRel(err);
}

/**
 * Sets module's angle, relative to robot and relative to current position
 * 
 * @param target double input for module relative angle target
 */
void SwerveModule::setModuleAngleRel(double target)
{

    double d_error = (target - prevErr) / 0.02;
    prevErr = target;

    if (abs(target) < kIz)
    {
        iAcc += target / 0.02;
    }
    else
    {
        iAcc = 0;
    }

    //double command = (kP * target) + (kI * iAcc) + (kD * d_error);
    double command = ((SmartDashboard::GetNumber(moduleID + "_P", 0.01)) * target) + ((SmartDashboard::GetNumber(moduleID + "_I", 0.00000)) * iAcc) + ((SmartDashboard::GetNumber(moduleID + "_D", 0.0001)) * d_error);
    if (d_error < 0.5 && abs(target) > 10.0)
    {
        command *= 2.0;
    }
    frc::SmartDashboard::PutNumber(moduleID + "command", command);
    rotationMotor._Set(command);
}

void SwerveModule::testTurning(double power)
{
    rotationMotor._Set(power);
}

/**
 * Returns if module is at the targeted position
 * 
 * @return if the module is at targeted position
 */
bool SwerveModule::angleOnTarget()
{
    bool onTarget = abs(angleTarget - getModuleAngle()) < angleTol;
    return onTarget;
}

void SwerveModule::ConfigureMotors()
{
    rotationMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void SwerveModule::SetIdleMode(rev::CANSparkMax::IdleMode idlemode)
{
    rotationMotor.SetIdleMode(idlemode);
}

void SwerveModule::UpdateTelemetry()
{
    SmartDashboard::PutNumber(moduleID + "_P", (SmartDashboard::GetNumber(moduleID + "_P", 0.01)));
    SmartDashboard::PutNumber(moduleID + "_I", (SmartDashboard::GetNumber(moduleID + "_I", 0.00000)));
    SmartDashboard::PutNumber(moduleID + "_D", (SmartDashboard::GetNumber(moduleID + "_D", 0.0001)));
}