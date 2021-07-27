#pragma once

#include <math.h>
#include "enums/types.hpp"
#include "sleep.hpp"
#include "connection.hpp"

namespace KSP
{
    class NodeExecutor
    {
    private:
        ManeuverNode m_node;
        Vessel m_vessel;
    public:
        NodeExecutor(ManeuverNode node, Vessel vessel);
        ~NodeExecutor();
    public:
        void execute(Connection connection, double throttle);
    private:
        double get_burn_time(double throttle, double delta_v_factor = 1.0);
    };

    NodeExecutor::NodeExecutor(ManeuverNode node, Vessel vessel) : m_node(node), m_vessel(vessel)
    {

    }

    NodeExecutor::~NodeExecutor()
    {

    }

    void NodeExecutor::execute(Connection connection, double throttle)
    {
        /* Get burn times. */
        auto burn_time = get_burn_time(throttle);

        if (burn_time < 2.0)
        {
            throttle = get_burn_time(2.0);
            burn_time = get_burn_time(throttle);
        }

        auto lead_burn_time = get_burn_time(throttle, 0.5);
        auto burn_start_time = m_node.ut() - lead_burn_time;
        auto burn_stop_time = m_node.ut() + (burn_time - lead_burn_time);

        auto ut_call = connection.space_center.ut_call();
        auto ut_stream = connection.space_center.ut_stream();

        /* Warp 60s until burn. */
        connection.space_center.warp_to(burn_start_time - 60.0, 100000.0F, 4.0F);

        /* Enable autopilot. */
        sleep_milliseconds(100);
        m_vessel.auto_pilot().set_sas(false);
        sleep_milliseconds(100);
        m_vessel.auto_pilot().engage();
        sleep_milliseconds(100);
        m_vessel.auto_pilot().set_reference_frame(m_vessel.orbital_reference_frame());
        sleep_milliseconds(100);

        /* Target node burn vector. */
        while (ut_stream() < burn_start_time - 0.01)
        {
            m_vessel.auto_pilot().set_target_direction(m_node.remaining_burn_vector());
            KSP::sleep_milliseconds(10);
        }

        /* Throttle up. */
        m_vessel.control().set_throttle(throttle);

        /* Target node burn vector. */
        while (ut_stream() < burn_stop_time - 0.51)
        {
            m_vessel.auto_pilot().set_target_direction(m_node.remaining_burn_vector());
            KSP::sleep_milliseconds(10);
        }

        /* Throttle down. */
        m_vessel.control().set_throttle(0);
        burn_stop_time = get_burn_time(throttle * 0.5) + ut_stream();
        m_vessel.control().set_throttle(throttle * 0.5);

        /* Target node burn vector. */
        while (ut_stream() < burn_stop_time - 0.001)
        {
            m_vessel.auto_pilot().set_target_direction(m_node.remaining_burn_vector());
            KSP::sleep_milliseconds(1);
        }

        /* Cut engines, disable autopilot, remove node. */
        m_vessel.control().set_throttle(0);
        sleep_milliseconds(100);
        m_vessel.auto_pilot().disengage();
        sleep_milliseconds(100);
        m_node.remove();
    }

    double NodeExecutor::get_burn_time(double throttle, double delta_v_factor)
    {
        auto g = 9.80665;
        auto isp = m_vessel.specific_impulse();
        auto mass = m_vessel.mass();
        auto mass_delta = mass - mass / pow(M_E, (m_node.remaining_delta_v() * delta_v_factor) / (g * isp));

        return mass_delta * g * isp / (m_vessel.available_thrust() * throttle);
    }
}



