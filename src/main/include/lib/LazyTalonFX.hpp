#pragma once

#include <ctre/Phoenix.h>
using namespace ctre;
using namespace frc;

class LazyTalonFX : public WPI_TalonFX
{
protected:
    double mLastSet = 420.0;
    ControlMode mLastControlMode;
    const char* device = "Talon FX";

public:
    LazyTalonFX() = delete;
    LazyTalonFX(int ID) : BaseMotorController(ID, device)
    , BaseTalon(ID, device), TalonFX(ID)
    , WPI_BaseMotorController(ID, device)
    , WPI_TalonFX(ID) 
    {          
        ConfigFactoryDefault();
    }

public:
    /**
     * Sets Talon's control mode and associated value
     * 
     * @param mode is the control mode of the Talon
     * @param value is the input value based on the control mode
     */ 
    void _Set(ControlMode mode, double value) 
    {
        if ((value != mLastSet) || (mode != mLastControlMode))
        {
            mLastSet = value;
            mLastControlMode = mode;
            Set(mode, value);
        }
    }
};