#pragma once

#include <cmath>

class Vector2d
{
    #define _USE_MATH_DEFINES
    protected:
    double x;
    double y;
    double ang; // -180 ~ 180 with (0, 1) being 0

    public:
    Vector2d() : x(0.0), y(0.0)
    {
        ang = 420.0;
    }

    Vector2d(double x, double y) : x(x), y(y)
    {
        ang = (-atan2(x, y) * (180.0/M_PI)) + 90.0;
        if(ang > 180.0)
            ang -= 360;
        else if(ang < -180.0)
            ang += 360.0;
    }

    Vector2d(Vector2d &in) : x(in.x), y(in.y), ang(in.ang)
    {}

    double getX() const
    {
        return x;
    }

    double getY() const
    {
        return y;
    }

    double getAng() const
    {
        return ang;
    }
};