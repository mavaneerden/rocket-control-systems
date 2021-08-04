#pragma once

#include <math.h>
#include "enums/types.hpp"

namespace KSP
{
    double rad_to_deg(double radians)
    {
        return radians * (180 / M_PI);
    }

    double deg_to_rad(double degrees)
    {
        return degrees * (M_PI / 180);
    }
}