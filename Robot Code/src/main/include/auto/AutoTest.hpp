#pragma once

#include <frc/Timer.h>

#include "AutoInterface.hpp"
#include "Robotmap.hpp"
#include "lib/TrajectoryReader.hpp"

class AutoTest : public AutoInterface {
 public:
    // Name of this program, used by SmartDash
    static std::string GetName();

 private:
    // Get a referance to the robotmap
    Robotmap& IO;

    // State Variables
    int m_state;   
    Timer m_autoTimer;

    void nextState();

 public:
    // Constructor requires a reference to the RobotMap
    AutoTest() = delete;
    AutoTest(Robotmap &);
    ~AutoTest();

    // Auto Program Logic
    void Run();
    void UpdateSmartDash();
};