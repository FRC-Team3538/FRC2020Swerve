#pragma once

#include "Subsystem.hpp"
#include "SwerveModule.hpp"
#include "Constants.hpp"
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

    enum encID
    {
        frEnc = 0,
        flEnc,
        brEnc,
        blEnc
    };

    const double poleDeadband = 4.0 * (Constants::pi / 180.0);// Radians

    SwerveModule frontRight{"0", motorID::frRotate, motorID::frDrive, encID::frEnc, Constants::frontRight, 0.0};
    SwerveModule frontLeft{"1", motorID::flRotate, motorID::flDrive, encID::flEnc, Constants::frontLeft, 0.0};
    SwerveModule backRight{"2", motorID::brRotate, motorID::brDrive, encID::brEnc, Constants::backRight, 0.0};
    SwerveModule backLeft{"3", motorID::blRotate, motorID::blDrive, encID::blEnc, Constants::backLeft, 0.0};

    SwerveKinematics SK{};
    SwerveKinematics::twoDArr motorVals;

    PigeonIMU pidgeotto{8};

    double prevTheta = -420.0;

    double deadbandPoles(double input);

public:
    SwerveController();

    void UpdateTelemetry();
    void ConfigureMotors();

    void SwerveDrive(double x, double y, double rotate, bool fieldCentric);
    void TestSteering(double x, double y);
    void SetIdleMode(rev::CANSparkMax::IdleMode idlemode);

    double GetAngle();
    void SetAngle(double angle);
};