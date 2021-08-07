#include "../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto node = vessel.control().nodes().at(0);
    auto executor = KSP::NodeExecutor(node, vessel);

    executor.execute(connection, 1.0);

    KSP::sleep_seconds(1);

    vessel.control().activate_next_stage();

    return 0;
}
