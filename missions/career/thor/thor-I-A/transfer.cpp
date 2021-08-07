#include "../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto target_vessel = connection.space_center.target_vessel();
    auto maneuver = KSP::Maneuver(connection, vessel);

    /* Match inclination to target. */
    maneuver.change_inclination(target_vessel);
    KSP::sleep_seconds(5);

    /* Transfer to target. */
    maneuver.transfer_to_vessel(target_vessel);
    KSP::sleep_seconds(5);

    /* Circularise at target. */
    maneuver.cicularize(true);
}

