#pragma once

#include <string>
#include <unordered_map>

namespace KSP
{
    namespace resources
    {
        const std::string ELECTRIC_CHARGE = "ElectricCharge";
        const std::string LIQUID_FUEL = "LiquidFuel";
        const std::string OXIDIZER = "Oxidizer";
        const std::string INTAKE_AIR = "IntakeAir";
        const std::string SOLID_FUEL = "SolidFuel";
        const std::string MONOPROPELLANT = "MonoPropelant";
        const std::string XENON_GAS = "XenonGas";
        const std::string ORE = "Ore";
        const std::string ABLATOR = "Ablator";

        const std::unordered_map<std::string, double> densities({
            {ELECTRIC_CHARGE, 0},
            {LIQUID_FUEL, 5},
            {OXIDIZER, 5},
            {SOLID_FUEL, 7.5},
            {ABLATOR, 1}
        });
    };
}
