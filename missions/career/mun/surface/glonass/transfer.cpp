#include "../../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto body = vessel.orbit().body();
    auto mun = connection.get_body(KSP::bodies::MUN);

    /* Create and execute circularisation maneuver node. */
    auto maneuver = KSP::Maneuver(connection, vessel);

    maneuver.change_inclination(mun);
    KSP::sleep_seconds(3);
    maneuver.transfer_to_body(mun);
}