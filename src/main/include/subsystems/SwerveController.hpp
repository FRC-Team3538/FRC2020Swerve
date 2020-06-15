#pragma once

#include "Subsystem.hpp"
#include "SwerveModule.hpp"
#include "Constants.hpp"
#include <vector>
#include "lib/SwerveKinematics.hpp"

using namespace std;

class SwerveController : public Subsystem
{
private:
    enum motorID
    {
        frDrive = 0,
        frRotate,
        flDrive,
        flRotate,
        brDrive,
        brRotate,
        blDrive,
        blRotate
    };

    SwerveModule frontRight{'0', motorID::frRotate, motorID::frDrive, Constants::frontRight, 0.0};
    SwerveModule frontLeft{'1', motorID::flRotate, motorID::flDrive, Constants::frontLeft, 0.0};
    SwerveModule backRight{'2', motorID::brRotate, motorID::brDrive, Constants::backRight, 0.0};
    SwerveModule backLeft{'3', motorID::blRotate, motorID::blDrive, Constants::backLeft, 0.0};

    SwerveKinematics SK{};
    SwerveKinematics::twoDArr motorVals;

    PigeonIMU pigeotto{8};

    void Controller();

public:
    SwerveController();

    void UpdateTelemetry();
    void ConfigureMotors();

    void SwerveDrive(double x, double y, double rotate, bool fieldCentric);

    double GetAngle();
};