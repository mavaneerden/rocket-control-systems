#pragma once

#include <math.h>
#include "enums/types.hpp"
#include "sleep.hpp"
#include "connection.hpp"
#include "constants.hpp"
#include "stages.hpp"
#include "formulae.hpp"

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
        double get_burn_time_stage(int stage, double throttle, double delta_v_remaining);
    };

    NodeExecutor::NodeExecutor(ManeuverNode node, Vessel vessel) : m_node(node), m_vessel(vessel)
    {

    }

    NodeExecutor::~NodeExecutor()
    {

    }

    void NodeExecutor::execute(Connection connection, double throttle)
    {
        if (m_node.delta_v() < 0.05)
        {
            return;
        }

        /* Get burn times. */
        auto current_stage = get_current_decouple_stage(m_vessel);
        auto stage_total_burn_time = get_decouple_stage_burn_time(current_stage, m_vessel, throttle, connection);
        auto stage_total_delta_v = get_decouple_stage_delta_v(current_stage, m_vessel, throttle);
        auto remaining_delta_v = m_node.remaining_delta_v();
        auto lead_delta_v = remaining_delta_v / 2;
        auto burn_time = get_burn_time_stage(current_stage, throttle, remaining_delta_v);
        auto decouple_index = 0;
        auto decouple_at = std::vector<double>();
        auto total_burn_time = 0.0;
        auto lead_burn_time = 0.0;

        while(stage_total_burn_time <= burn_time)
        {
            remaining_delta_v -= stage_total_delta_v;

            if (remaining_delta_v >= lead_delta_v)
            {
                lead_burn_time += get_burn_time_stage(current_stage, throttle, stage_total_delta_v);
            }
            else if (remaining_delta_v + stage_total_delta_v >= lead_delta_v)
            {
                lead_burn_time += get_burn_time_stage(current_stage, throttle, remaining_delta_v + stage_total_delta_v - lead_delta_v);
            }

            current_stage--;
            total_burn_time += stage_total_burn_time;
            decouple_at.push_back(total_burn_time);

            stage_total_burn_time = get_decouple_stage_burn_time(current_stage, m_vessel, throttle, connection);
            burn_time = get_burn_time_stage(current_stage, throttle, remaining_delta_v);
            stage_total_delta_v = get_decouple_stage_delta_v(current_stage, m_vessel, throttle);
        }

        total_burn_time += burn_time;
        remaining_delta_v -= stage_total_delta_v;

        if (remaining_delta_v >= lead_delta_v)
        {
            lead_burn_time += get_burn_time_stage(current_stage, throttle, stage_total_delta_v);
        }
        else if (remaining_delta_v + stage_total_delta_v >= lead_delta_v)
        {
            lead_burn_time += get_burn_time_stage(current_stage, throttle, remaining_delta_v + stage_total_delta_v - lead_delta_v);
        }

        auto burn_start_time = m_node.ut() - lead_burn_time;
        auto burn_stop_time = m_node.ut() + (total_burn_time - lead_burn_time);

        auto ut_call = connection.space_center.ut_call();
        auto ut_stream = connection.space_center.ut_stream();
        auto current_stage_stream = m_vessel.control().current_stage_stream();
        auto remaining_delta_v_stream = m_node.remaining_delta_v_stream();
        auto remaining_vector_stream = m_node.remaining_burn_vector_stream();

        for (size_t i = 0; i < decouple_at.size(); i++)
        {
            decouple_at[i] += burn_start_time;
        }

        std::cout << "TOTAL TIME: " << total_burn_time << std::endl;
        std::cout << "LEAD TIME:  " << lead_burn_time << std::endl;
        std::cout << "START TIME: " << burn_start_time - ut_stream() << std::endl;
        std::cout << "STOP TIME:  " << burn_stop_time - ut_stream() << std::endl;

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
            m_vessel.auto_pilot().set_target_direction(remaining_vector_stream());
            KSP::sleep_milliseconds(10);
        }

        /* Throttle up. */
        m_vessel.control().set_throttle(throttle);

        /* Target node burn vector. */
        while (ut_stream() < burn_stop_time - 1)
        {
            auto stage = current_stage_stream();

            m_vessel.auto_pilot().set_target_direction(remaining_vector_stream());

            if (
                decouple_index < decouple_at.size()
                && ut_stream() >= decouple_at[decouple_index]
            ) {
                m_vessel.control().activate_next_stage();
                decouple_index++;
            }

            KSP::sleep_milliseconds(10);
        }

        // remaining_delta_v = remaining_delta_v_stream();
        // auto formula_constant = 10;
        // auto twr = get_twr(m_vessel, m_vessel.orbit().body());

        // while (remaining_delta_v_stream() > 0.5)
        // {
        //     auto throttle = std::min(1.0 / pow(M_E, -0.4 * (twr / 5) * (remaining_delta_v - formula_constant)), 1.0);
        //     m_vessel.control().set_throttle(throttle);
        //     m_vessel.auto_pilot().set_target_direction(remaining_vector_stream());

        //     std::cout << throttle << std::endl;

        //     remaining_delta_v = remaining_delta_v_stream();

        //     KSP::sleep_milliseconds(10);
        // }


        /* Throttle down. */
        m_vessel.control().set_throttle(0);
        burn_stop_time = get_burn_time(throttle * 0.5) + ut_stream();
        m_vessel.control().set_throttle(throttle * 0.5);

        /* Target node burn vector. */
        while (ut_stream() < burn_stop_time - 0.001)
        {
            m_vessel.auto_pilot().set_target_direction(remaining_vector_stream());
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
        auto g = STANDARD_GRAVITY;
        auto isp = m_vessel.specific_impulse();
        auto mass = m_vessel.mass();
        auto mass_delta = mass - mass / pow(M_E, (m_node.remaining_delta_v() * delta_v_factor) / (g * isp));

        return mass_delta * g * isp / (m_vessel.available_thrust() * throttle);
    }

    double NodeExecutor::get_burn_time_stage(int stage, double throttle, double delta_v_remaining)
    {
        auto g = STANDARD_GRAVITY;
        auto isp = get_decouple_stage_isp(stage, m_vessel, throttle);
        auto mass = get_decouple_stage_mass(stage, m_vessel) + get_decouple_stage_mass_above(stage, m_vessel);
        auto mass_delta = mass - mass / pow(M_E, (delta_v_remaining) / (g * isp));

        return mass_delta * g * isp / (get_decouple_stage_thrust(stage, m_vessel, throttle) * throttle);
    }
}



