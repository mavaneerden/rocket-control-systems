#include "../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();

    auto vessel = connection.space_center.active_vessel();
    auto body_reference_frame = vessel.orbit().body().reference_frame();
    auto reference_frame = vessel.surface_reference_frame();
    auto target_vector = KSP::Vector3(1, 0, 0);
    auto parachute_altitude = 1500;

    /* Event for opening the parachute. */
    auto surface_altitude_call = vessel.flight(reference_frame).surface_altitude_call();
    auto parachute_event = connection.krpc.add_event(
        KSP::Expression::less_than_or_equal(
            connection.client,
            KSP::Expression::call(connection.client, surface_altitude_call),
            KSP::Expression::constant_double(connection.client, parachute_altitude)
        )
    );

    /* Event for reaching apogee. */
    auto vertical_speed_call = vessel.flight(body_reference_frame).vertical_speed_call();
    auto vertical_speed_event = connection.krpc.add_event(
        KSP::Expression::less_than_or_equal(
            connection.client,
            KSP::Expression::call(connection.client, vertical_speed_call),
            KSP::Expression::constant_double(connection.client, 0)
        )
    );

    /* Set auto pilot variables. */
    vessel.auto_pilot().set_reference_frame(reference_frame);
    vessel.auto_pilot().set_target_direction(target_vector.to_tuple());
    vessel.auto_pilot().engage();

    /* Collect science with Mystery Goo unit for launch pad. */
    vessel.control().set_action_group(1, true);

    /* Wait for console input. */
    KSP::wait_for_user();

    /* Countdown to launch. */
    KSP::countdown_seconds(3, "LAUNCH IN", "LAUNCH");

    /* Activate SRB, burns for 18 seconds. */
    vessel.control().activate_next_stage();
    KSP::sleep_seconds(5);

    /* Collect science with Mystery Goo unit for flying low. */
    vessel.control().set_action_group(2, true);

    /* Wait until apogee. */
    vertical_speed_event.acquire();
    vertical_speed_event.wait();
    vertical_speed_event.release();

    /* Wait until parachute deploy. */
    parachute_event.acquire();
    parachute_event.wait();
    parachute_event.release();

    /* Open parachute. */
    vessel.control().activate_next_stage();
}