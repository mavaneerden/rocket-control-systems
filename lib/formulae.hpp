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

    double get_twr(Vessel vessel, KSP::Body body)
    {
        return vessel.thrust() / (vessel.mass() * get_g_at_altitude(body, vessel.flight().mean_altitude()));
    }

    double get_burn_time(Vessel vessel, double delta_v, double throttle, double delta_v_factor = 1.0)
    {
        auto g = STANDARD_GRAVITY;
        auto isp = vessel.specific_impulse();
        auto mass = vessel.mass();
        auto mass_delta = mass - mass / pow(M_E, (delta_v * delta_v_factor) / (g * isp));

        return mass_delta * g * isp / (vessel.available_thrust() * throttle);
    }
}