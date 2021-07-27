#pragma once

#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>

namespace KSP
{
    typedef krpc::services::KRPC::Expression Expression;
    typedef krpc::services::SpaceCenter::SASMode SASMode;
    typedef krpc::services::SpaceCenter::Orbit Orbit;
    typedef krpc::services::SpaceCenter::CelestialBody Body;
    typedef krpc::services::SpaceCenter::Vessel Vessel;
    typedef krpc::services::SpaceCenter::Node ManeuverNode;
}
