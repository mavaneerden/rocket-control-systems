#include "../../lib/ksp.hpp"
#include "launch.hpp"
#include "landing.hpp"

/**
 * TODO:
 * - Capsule retrorockets
 * - Abort detection (twr < 1, incorrect attitude)
 * - Cancel horizontal speed on ascent
 * - Target lon/lat using aerosurfaces (coarse targeting)
 * - Hover & cancel horizontal velocity
 * - Target lon/lat using engines (fine targeting)
 */
int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();

    /* Launch the vessel to space. */
    // launch(connection);
    /* Land the booster and capsule. */
    landing(connection);
}