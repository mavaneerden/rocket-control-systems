#pragma once

#include <unordered_set>
#include <math.h>
#include "connection.hpp"
#include "enums/types.hpp"
#include "constants.hpp"
#include "enums/resources.hpp"

namespace KSP
{
    double get_decouple_stage_mass(int stage, Vessel vessel)
    {
        auto parts = vessel.parts().in_decouple_stage(stage);
        double mass = 0.0;

        for (auto part : parts)
        {
            mass += part.mass();
        }

        return mass;
    }

    double get_decouple_stage_mass_above(int stage, Vessel vessel)
    {
        double mass = 0.0;

        for (int i = -1; i < stage; i++)
        {
            mass += get_decouple_stage_mass(i, vessel);
        }

        return mass;
    }

    double get_decouple_stage_dry_mass(int stage, Vessel vessel)
    {
        auto parts = vessel.parts().in_decouple_stage(stage);
        double mass = 0.0;

        for (auto part : parts)
        {
            mass += part.dry_mass();
        }

        return mass;
    }

    double get_decouple_stage_dry_mass_above(int stage, Vessel vessel)
    {
        double mass = 0.0;

        for (int i = -1; i < stage; i++)
        {
            mass += get_decouple_stage_dry_mass(i, vessel);
        }

        return mass;
    }


    std::unordered_map<int32_t, double> get_all_decouple_stage_masses(Vessel vessel)
    {
        auto current_stage = vessel.control().current_stage();
        auto parts = vessel.parts().all();
        auto map = std::unordered_map<int32_t, double>();

        for (auto part : parts)
        {
            map[part.decouple_stage()] += part.mass();
        }

        return map;
    }

    std::unordered_map<int32_t, double> get_all_stage_masses(Vessel vessel)
    {
        auto current_stage = vessel.control().current_stage();
        auto parts = vessel.parts().all();
        auto map = std::unordered_map<int32_t, double>();

        for (auto part : parts)
        {
            map[part.stage()] += part.mass();
        }

        return map;
    }

    double get_decouple_stage_thrust(int stage, Vessel vessel, double throttle)
    {
        auto engines = vessel.parts().engines();
        double thrust = 0.0;

        for (auto engine : engines)
        {
            if (engine.part().decouple_stage() == stage)
            {
                thrust += engine.available_thrust() * throttle;
            }
        }

        return thrust;
    }

    double get_stage_mass_flow(int stage, Vessel vessel, double throttle)
    {
        auto engines = vessel.parts().engines();
        double mass_flow = 0.0;

        for (auto engine : engines)
        {
            if (engine.part().stage() == stage)
            {
                mass_flow += engine.max_vacuum_thrust() * throttle / engine.vacuum_specific_impulse();
            }
        }

        return mass_flow;
    }

    double get_decouple_stage_mass_flow(int stage, Vessel vessel, double throttle)
    {
        auto engines = vessel.parts().engines();
        double mass_flow = 0.0;

        for (auto engine : engines)
        {
            if (engine.part().decouple_stage() == stage)
            {
                mass_flow += engine.max_vacuum_thrust() * throttle / engine.vacuum_specific_impulse();
            }
        }

        return mass_flow;
    }

    double get_decouple_stage_isp(int stage, Vessel vessel, double throttle)
    {
        double thrust = get_decouple_stage_thrust(stage, vessel, throttle);
        double mass_flow = get_decouple_stage_mass_flow(stage, vessel, throttle);

        return thrust / mass_flow;
    }

    double get_decouple_stage_delta_v(int stage, Vessel vessel, double throttle)
    {
        double isp = get_decouple_stage_isp(stage, vessel, throttle);
        double mass_above = get_decouple_stage_mass_above(stage, vessel);
        double mass = get_decouple_stage_mass(stage, vessel) + mass_above;
        double dry_mass = get_decouple_stage_dry_mass(stage, vessel) + mass_above;

        return STANDARD_GRAVITY * isp * log(mass / dry_mass);
    }

    double get_current_decouple_stage(Vessel vessel)
    {
        auto decouple_stage = -1;

        for (auto part : vessel.parts().all())
        {
            auto part_stage = part.decouple_stage();

            if (part_stage > decouple_stage)
            {
                decouple_stage = part_stage;
            }
        }

        return decouple_stage;
    }

    /* Stage burn time in vacuum. */
    double get_decouple_stage_burn_time(int stage, Vessel vessel, double throttle, Connection connection)
    {
        auto mass_flow = get_decouple_stage_mass_flow(stage, vessel, throttle);
        auto resources = vessel.resources_in_decouple_stage(stage, false);
        auto solid_fuel_mass = resources.amount(resources::SOLID_FUEL) * resources::densities.at(resources::SOLID_FUEL);
        auto liquid_fuel_mass = resources.amount(resources::LIQUID_FUEL) * resources::densities.at(resources::LIQUID_FUEL);
        auto oxidizer_mass = resources.amount(resources::OXIDIZER) * resources::densities.at(resources::OXIDIZER);

        // TODO: what if fuel/ox ratio is incorrect?
        auto propellant_mass = solid_fuel_mass > 0 ? solid_fuel_mass : liquid_fuel_mass + oxidizer_mass;

        return propellant_mass * STANDARD_GRAVITY / mass_flow;
    }
}