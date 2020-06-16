#pragma once

#include <rev/CANSparkMax.h>
using namespace rev;
using namespace frc;

class LazySparkMax : public CANSparkMax
{
protected:
    double mLastSet = 420.0;

public:
    LazySparkMax() = delete;
    LazySparkMax(int ID) : CANSparkMax(ID, CANSparkMax::MotorType::kBrushless)
    {          
        RestoreFactoryDefaults();
    }

public:
    /**
     * Sets Talon's control mode and associated value
     * 
     * @param value is the input value based on the control mode
     */ 
    void _Set(double value) 
    {
        if ((value != mLastSet))
        {
            mLastSet = value;
            Set(value);
        }
    }
};