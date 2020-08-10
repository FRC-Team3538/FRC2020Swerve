#include "auto/AutoTest.hpp"

// Name for Smart Dash Chooser
std::string AutoTest::GetName()
{
    return "1 - Test";
}

// Initialization
// Constructor requires a reference to the robot map
AutoTest::AutoTest(Robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

AutoTest::~AutoTest() {}

//State Machine
void AutoTest::nextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoTest::Run()
{
    switch (m_state)
    {
    case 0:
    {
        cout << "woo";
    }
    break;

    default:
        cout << "woo";
    }

    UpdateSmartDash();
}

void AutoTest::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}