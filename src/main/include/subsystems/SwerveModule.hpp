#pragma once

#include "Subsystem.hpp"
#include "lib/Vector2d.hpp"
#include "lib/LazyTalonFX.hpp"
#include <rev/CANSparkMax.h>

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
    CANSparkMax rotationMotor;
    double encOffset;

public:
    SwerveModule &operator=(SwerveModule const &);   
    SwerveModule(char moduleID, int rotID, int driveID, const Vector2d& position, double encOffset);

    void ConfigureMotors();
    void UpdateTelemetry();

    double getModuleAngle();
    void setModuleAngle(double target);
    void setModule(double input);
    bool angleOnTarget();
};