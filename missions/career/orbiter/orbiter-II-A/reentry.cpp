#include "../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();

    /* Reference frames. */
    auto surface_reference_frame = vessel.surface_reference_frame();
    auto surface_velocity_reference_frame = vessel.surface_velocity_reference_frame();

    /* Targets. */
    auto parachute_altitude = 1500;
    auto retrograde_direction = KSP::Vector3(0, -1, 0);

    /* Event for opening the parachute. */
    auto surface_altitude_call = vessel.flight(surface_reference_frame).surface_altitude_call();
    auto parachute_event = connection.krpc.add_event(
        KSP::Expression::less_than_or_equal(
            connection.client,
            KSP::Expression::call(connection.client, surface_altitude_call),
            KSP::Expression::constant_double(connection.client, parachute_altitude)
        )
    );

    /* Orient spacecraft towards surface retrograde. */
    vessel.auto_pilot().set_reference_frame(surface_velocity_reference_frame);
    vessel.auto_pilot().set_target_direction(retrograde_direction.to_tuple());

    /* Wait until parachute deploy. */
    parachute_event.acquire();
    parachute_event.wait();
    parachute_event.release();

    /* Open parachute. */
    vessel.control().activate_next_stage();
}