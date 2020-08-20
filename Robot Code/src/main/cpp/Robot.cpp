/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"
#include <wpi/raw_ostream.h>

void Robot::RobotInit() {
  Configuration c;
  auto t = c.TryGet<TestConfig>("testConfig.json");
  wpi::outs() << "t.a: " << t.a << "\nt.cpc: " << t.cpc.toString("t.cpc")
              << "t.fx: " << t.fx.toString("t.fx") << "\n";

  TestConfig conf;
  try {
    c.Put<TestConfig>("written_testConfig.json", conf);
  } catch (int i) {
    wpi::outs() << "Error while writing: " << strerror(i) << "\n";
  }
}

// void Robot::RobotPeriodic()
//{
//    IO.SC.UpdateTelemetry();
//}
//
// void Robot::DisabledPeriodic()
//{
//    IO.SC.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
//}
//
// void Robot::AutonomousInit() {}
// void Robot::AutonomousPeriodic() {}
//
// void Robot::TeleopInit()
//{
//    IO.SC.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
//}
//
// void Robot::TeleopPeriodic()
//{
//    double forward = deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand), 0.05);
//    double strafe = deadband(IO.ds.Driver.GetX(GenericHID::kLeftHand), 0.05);
//    double rotate = deadband(IO.ds.Driver.GetX(GenericHID::kRightHand), 0.05);
//
//    IO.SC.TestSteering(strafe, forward);
//
//    IO.SC.SwerveDrive(strafe, forward, rotate, true);
//}
//
// void Robot::TestInit() {}
// void Robot::TestPeriodic() {}

double Robot::deadband(double input, double deadband) {
  if ((std::abs(input)) < deadband) {
    return 0.0;
  } else if (input > 0.95) {
    return 1.0;
  } else if (input < -0.95) {
    return -1.0;
  } else {
    return input;
  }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
