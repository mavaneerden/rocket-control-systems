#pragma once

#include <math.h>
#include "enums/types.hpp"

namespace KSP
{
    double get_g_at_altitude(KSP::Body body, double altitude)
    {
        return body.gravitational_parameter() / pow(body.equatorial_radius() + altitude, 2);
    }

    double get_vertical_acceleration(double thrust, double mass, KSP::Body body, double altitude)
    {
        return thrust / mass - get_g_at_altitude(body, altitude);
    }

    double get_twr(double thrust, double mass, KSP::Body body, double altitude)
    {
        return thrust / (mass * get_g_at_altitude(body, altitude));
    }
}