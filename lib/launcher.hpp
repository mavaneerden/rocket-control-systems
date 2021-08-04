#pragma once

#include "angles.hpp"

namespace KSP
{
    const double TURN_SPEED = 120;

    class Launcher
    {
    private:
    private:
        Vessel m_vessel;
        ResourcesMap m_resources;
        double m_inclination;
        double m_turn_altitude = 0.0;
        bool m_orbit;
    public:
        Launcher(Vessel vessel, ResourcesMap resources, double inclination = 0.0, bool orbit = true);
        ~Launcher();
    public:
        void launch(double throttle = 1.0, int countdown = 0);
        bool step(int current_stage, double altitude, double speed);
    private:
        double altitude_function_derivative(double altitude);
    };

    Launcher::Launcher(Vessel vessel, ResourcesMap resources, double inclination, bool orbit)
        : m_vessel(vessel), m_resources(resources), m_inclination(inclination), m_orbit(orbit)
    {
    }

    Launcher::~Launcher()
    {
    }

    void Launcher::launch(double throttle, int countdown)
    {
        /* Countdown to launch. */
        if (countdown > 0)
        {
            KSP::countdown_seconds(countdown, "LAUNCH IN", "LAUNCH");
        }

        /* Activate first stage. */
        m_vessel.control().set_throttle(throttle);
        m_vessel.control().activate_next_stage();
    }

    bool Launcher::step(int stage, double altitude, double speed)
    {
        /* Set turn altitude if speed is above a limit and craft goes to orbit. */
        if (m_orbit && speed > TURN_SPEED && m_turn_altitude <= 0.01)
        {
            m_turn_altitude = altitude;
        }

        /* Set target pitch and heading. */
        auto target_pitch = m_turn_altitude > 0.01 ? std::max(0.0, rad_to_deg(atan2(altitude, altitude_function_derivative(altitude) * altitude))) : 90;
        auto target_heading = 90 + m_inclination;

        m_vessel.auto_pilot().target_pitch_and_heading(target_pitch, target_heading);

        /* Stage when fuel is low. */
        if (
            m_resources.find(stage) != m_resources.end()
            && m_resources[stage]() < 0.1
        ) {
            m_vessel.control().activate_next_stage();
            return true;
        }

        return false;
    }

    double Launcher::altitude_function_derivative(double altitude)
    {
        return (altitude - m_turn_altitude) / (10000 - m_turn_altitude);
    }
}