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
    : moduleID(moduleID), position(position), driveMotor(LazyTalonFX(driveID)), rotationMotor(CANSparkMax(rotID, CANSparkMax::MotorType::kBrushless)), encOffset(encOffset)
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
 * Sets Module Target Angle
 * 
 * @param target is the module's target angle, relative to the front of the robot
 */
void SwerveModule::setModuleAngle(double target)
{
}

/**
 * Sets Module Power Output from a scale of -1~1
 * 
 * @param input is the module's power output
 */
void SwerveModule::setModule(double input)
{
    driveMotor._Set(ControlMode::PercentOutput, input);
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