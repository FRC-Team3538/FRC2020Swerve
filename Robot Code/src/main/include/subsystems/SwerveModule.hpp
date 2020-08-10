#pragma once

#include "Subsystem.hpp"
#include "lib/Vector2d.hpp"
#include "lib/LazyTalonFX.hpp"
#include "lib/LazySparkMax.hpp"
#include <frc/DutyCycleEncoder.h>
#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;
using namespace frc;
using namespace rev;
using namespace ctre;

class SwerveModule : public Subsystem
{
private:
    string moduleID;
    const Vector2d& position;
    LazyTalonFX driveMotor;
    LazySparkMax rotationMotor;
    AnalogInput rotEnc;
    double encOffset;

    // static constexpr double kP = 0.035;
    // static constexpr double kI = 0.00001;
    // static constexpr double kD = 0.02;
    static constexpr double kIz = 10.0; // Degrees

    static constexpr double rotScaleFac = (360.0 / 3.3); // Scale factor to convert 3.3v analog signal to degrees

    static constexpr double angleTol = 3.0; // Degrees

    double iAcc = 0.0;
    double prevErr = 0.0;

    double angleTarget = 0.0;

public:

    SwerveModule &operator=(SwerveModule const &);   
    SwerveModule(string moduleID, int rotID, int driveID, int encID, const Vector2d& position, double encOffset);

    void ConfigureMotors();
    void SetIdleMode(rev::CANSparkMax::IdleMode idlemode);
    void UpdateTelemetry();

    double getModuleAngle();
    void setModule(double angle, double power);

    void testTurning(double power);

    void setModuleAngleRel(double target);
    bool angleOnTarget();
};