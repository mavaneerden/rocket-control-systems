#include "../../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto maneuver = KSP::Maneuver(connection, vessel);

    maneuver.cicularize(false);

    return 0;
}
