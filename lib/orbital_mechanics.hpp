#pragma once

#include <math.h>
#include "enums/types.hpp"
#include "vector3.hpp"

namespace KSP
{
    double calculate_velocity(Body body, double apoapsis, double periapsis, double altitude)
    {
        auto mu = body.gravitational_parameter();
        auto radius = body.equatorial_radius();
        auto sma = (apoapsis + periapsis) / 2 + radius;

        return sqrt(mu * (2 / (radius + altitude) - 1 / sma));
    }

    double calculate_velocity(Orbit orbit, double altitude)
    {
        return calculate_velocity(orbit.body(), orbit.apoapsis_altitude(), orbit.periapsis_altitude(), altitude);
    }

    KSP::Vector3 velocity_at(Orbit orbit, double ut)
    {
        auto a = orbit.semi_major_axis();
        auto e = orbit.eccentricity();
        auto aop = orbit.argument_of_periapsis();
        auto lan = orbit.longitude_of_ascending_node();
        auto i = orbit.inclination();
        auto mu = orbit.body().gravitational_parameter();

        auto M_approach = orbit.mean_anomaly_at_ut(ut);
        auto E_approach = orbit.eccentric_anomaly_at_ut(ut);
        auto V_approach = orbit.true_anomaly_at_ut(ut);
        auto d_approach = a * (1 - e * cos(E_approach));

        auto ov_approach = KSP::Vector3(-sin(E_approach), sqrt(1 - pow(e, 2)) * cos(E_approach), 0) * (sqrt(mu * a) / d_approach);

        return KSP::Vector3(
            ov_approach.m_x * (cos(aop)*cos(lan) - sin(aop)*cos(i)*sin(lan)) - ov_approach.m_y * (sin(aop)*cos(lan) + cos(aop)*cos(i)*sin(lan)),
            ov_approach.m_x * (sin(aop)*sin(i)) + ov_approach.m_y * (cos(aop)*sin(i)),
            ov_approach.m_x * (cos(aop)*sin(lan) + sin(aop)*cos(i)*cos(lan)) + ov_approach.m_y * (cos(aop)*cos(i)*cos(lan) - sin(aop)*sin(lan))
        );
    }
}
