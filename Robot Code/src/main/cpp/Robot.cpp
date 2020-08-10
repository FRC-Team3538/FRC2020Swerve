/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"

void Robot::RobotInit() {}

void Robot::RobotPeriodic()
{
    IO.SC.UpdateTelemetry();
}

void Robot::DisabledPeriodic()
{
    IO.SC.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    IO.SC.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Robot::TeleopPeriodic()
{
    double forward = deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand), 0.05);
    double strafe = deadband(IO.ds.Driver.GetX(GenericHID::kLeftHand), 0.05);
    double rotate = deadband(IO.ds.Driver.GetX(GenericHID::kRightHand), 0.05);

    //IO.SC.TestSteering(strafe, forward);

    IO.SC.SwerveDrive(strafe, forward, rotate, true);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

double Robot::deadband(double input, double deadband)
{
    if ((std::abs(input)) < deadband)
    {
        return 0.0;
    }
    else if (input > 0.95)
    {
        return 1.0;
    }
    else if (input < -0.95)
    {
        return -1.0;
    }
    else
    {
        return input;
    }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
