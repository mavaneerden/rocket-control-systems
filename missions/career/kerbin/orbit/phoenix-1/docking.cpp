#include "../../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto target_vessel = connection.space_center.target_vessel();
    auto vessel_reference_frame = vessel.orbital_reference_frame();

    std::cout << "Target the docking port." << std::endl;
    KSP::wait_for_user();
    std::cout << "Control the docking port." << std::endl;
    KSP::wait_for_user();

    auto port = vessel.parts().controlling().docking_port();
    auto target_port = connection.space_center.target_docking_port();
    auto target_reference_frame = target_port.reference_frame();
    auto target_vector_stream = target_port.position_stream(vessel_reference_frame);

    vessel.auto_pilot().set_reference_frame(vessel_reference_frame);
    vessel.auto_pilot().engage();

    while (port.state() != krpc::services::SpaceCenter::DockingPortState::docked)
    {
        vessel.auto_pilot().set_target_direction(target_vector_stream());

        KSP::sleep_milliseconds(100);
    }

}