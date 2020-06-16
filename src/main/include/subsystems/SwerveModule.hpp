#pragma once

#include "Subsystem.hpp"
#include "lib/Vector2d.hpp"
#include "lib/LazyTalonFX.hpp"
#include "lib/LazySparkMax.hpp"

using namespace std;
using namespace frc;
using namespace rev;
using namespace ctre;

class SwerveModule : public Subsystem
{
private:
    char moduleID;
    const Vector2d& position;
    LazyTalonFX driveMotor;
    LazySparkMax rotationMotor;
    double encOffset;

    static const double kP = 0.035;
    static const double kI = 0.00001;
    static const double kD = 0.02;
    static const double kIz = 10.0; // Degrees

    double iAcc = 0.0;
    double prevErr = 0.0;

public:
    SwerveModule &operator=(SwerveModule const &);   
    SwerveModule(char moduleID, int rotID, int driveID, const Vector2d& position, double encOffset);

    void ConfigureMotors();
    void UpdateTelemetry();

    double getModuleAngle();
    void setModule(double angle, double power);
    void setModuleAngleRel(double target);
    bool angleOnTarget();
};