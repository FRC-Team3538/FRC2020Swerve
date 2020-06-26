#pragma once

#include <cmath>

class Vector2d
{
    protected:
    double x;
    double y;

    public:
    Vector2d() : x(0.0), y(0.0)
    {}

    Vector2d(double x, double y) : x(x), y(y)
    {}

    Vector2d(Vector2d &in) : x(in.x), y(in.y)
    {}

};