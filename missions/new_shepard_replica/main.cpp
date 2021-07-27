#include "../../lib/ksp.hpp"
#include "launch.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();

    /* Launch the vessel to space. */
    launch(connection);

    KSP::sleep_seconds(5);
}