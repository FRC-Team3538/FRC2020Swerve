#pragma once

#include "Constants.hpp"

class SwerveKinematics
{
    // {frSpd, frAng}, {flSpd, frAng}, {brSpd, brAng}, {blSpd, brAng}
private:
    double wheelVals[4][2] =
        {
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0}};

public:
    // Yes, I did create a custom data type in order to return a 2d array rather
    // than requiring an input array that I could just edit
    typedef double (*twoDArr)[2];

    SwerveKinematics() {}


    /**
     * Calculates wheel kinematics based on controller input
     * 
     * @param x is Left-Right input from joystick
     * @param y is Forward-Backward input from joystick
     * @param rotate is joystick input to rotate robot
     */ 
    void Calculate(double x, double y, double rotate)
    {
        double A = x - (rotate * (Constants::wheelBase / Constants::diagonal));
        double B = x + (rotate * (Constants::wheelBase / Constants::diagonal));
        double C = y - (rotate * (Constants::trackWidth / Constants::diagonal));
        double D = y + (rotate * (Constants::trackWidth / Constants::diagonal));

        wheelVals[0][0] = sqrt(pow(B, 2) + pow(C, 2));
        wheelVals[1][0] = sqrt(pow(B, 2) + pow(D, 2));
        wheelVals[2][0] = sqrt(pow(A, 2) + pow(C, 2));
        wheelVals[3][0] = sqrt(pow(A, 2) + pow(D, 2));

        double max = wheelVals[0][0];
        if(wheelVals[1][0] > max)
        {
            max = wheelVals[1][0];
        }

        if(wheelVals[2][0] > max)
        {
            max = wheelVals[2][0];
        }

        if(wheelVals[3][0] > max)
        {
            max = wheelVals[3][0];
        }

        if(max > 1.0)
        {
            wheelVals[0][0] /= max;
            wheelVals[1][0] /= max;
            wheelVals[2][0] /= max;
            wheelVals[3][0] /= max;
        }

        wheelVals[0][1] = atan2(B, C) * (180/Constants::pi); 
        wheelVals[1][1] = atan2(B, D) * (180/Constants::pi); 
        wheelVals[2][1] = atan2(A, C) * (180/Constants::pi); 
        wheelVals[3][1] = atan2(A, D) * (180/Constants::pi); 
    }

    /**
     * Get results of Calculate() function
     * 
     * @return wheelVals is a 2d array with the following format:
     *         {frSpd, frAng}, {flSpd, frAng}, {brSpd, brAng}, {blSpd, brAng}
     */
    twoDArr GetKinematics()
    {
        return wheelVals;
    }
};