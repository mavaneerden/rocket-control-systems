#pragma once

#include "pid.hpp"
#include "formulae.hpp"
#include "enums/types.hpp"

namespace KSP
{
    class Lander
    {
    private:
        bool pid_started;
    public:
        Lander(Connection connection);
        ~Lander();
    public:
        double vertical_hoverslam_throttle(
            KSP::PID& pid_controller,
            KSP::Body body,
            double vessel_mass,
            double sea_level_altitude,
            double surface_altitude,
            double surface_speed,
            double ship_height,
            double target_height,
            double target_thrust,
            double drag = 0
        );
    };

    Lander::Lander(Connection connection) : pid_started(false)
    {
    }

    Lander::~Lander()
    {
    }

    double Lander::vertical_hoverslam_throttle(
        KSP::PID& pid_controller,
        KSP::Body body,
        double vessel_mass,
        double sea_level_altitude,
        double surface_altitude,
        double surface_speed,
        double ship_height,
        double target_height,
        double target_thrust,
        double drag
    ) {
        auto g = get_g_at_altitude(body, sea_level_altitude);
        auto a = (target_thrust) / vessel_mass - g;
        auto burn_altitude = pow(surface_speed, 2) / (2 * a);
        auto ship_altitude = surface_altitude - ship_height - target_height;
        auto altitude_delta = ship_altitude - burn_altitude;

        if (!pid_started)
        {
            if (altitude_delta < 0)
            {
                pid_controller.start();
                pid_started = true;
            }
            else
            {
                return 0;
            }
        }

        return std::max(0.0, std::min(1.0, pid_controller.step(0.0, altitude_delta)));
    }
}