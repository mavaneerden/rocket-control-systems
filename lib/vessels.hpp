#pragma once

#include "connection.hpp"
#include "enums/types.hpp"

namespace KSP
{
    KSP::Vessel get_vessel_by_name(std::string name, KSP::Connection connection)
    {
        auto vessels = connection.space_center.vessels();

        for (auto vessel : vessels)
        {
            if (vessel.name() == name)
            {
                return vessel;
            }
        }

        std::cout << "Vessel '" << name << "' not found, using active vessel instead.";

        return connection.space_center.active_vessel();
    }
}