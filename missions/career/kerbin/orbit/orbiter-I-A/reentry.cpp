#include "../../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();

    /* Reference frames. */
    auto surface_reference_frame = vessel.surface_reference_frame();
    auto orbit_reference_frame = vessel.orbital_reference_frame();
    auto surface_velocity_reference_frame = vessel.surface_velocity_reference_frame();

    /* Targets. */
    auto parachute_altitude = 1500;
    auto periapsis_target = 10000;
    auto retrograde_direction = KSP::Vector3(0, -1, 0);
    auto normal_direction = KSP::Vector3(0, 0, 1);

    /* Event for opening the parachute. */
    auto periapsis_altitude_call = vessel.orbit().periapsis_altitude_call();
    auto periapsis_event = connection.krpc.add_event(
        KSP::Expression::less_than_or_equal(
            connection.client,
            KSP::Expression::call(connection.client, periapsis_altitude_call),
            KSP::Expression::constant_double(connection.client, periapsis_target)
        )
    );

    /* Event for opening the parachute. */
    auto surface_altitude_call = vessel.flight(surface_reference_frame).surface_altitude_call();
    auto parachute_event = connection.krpc.add_event(
        KSP::Expression::less_than_or_equal(
            connection.client,
            KSP::Expression::call(connection.client, surface_altitude_call),
            KSP::Expression::constant_double(connection.client, parachute_altitude)
        )
    );

    /* Target retrograde. */
    vessel.auto_pilot().set_reference_frame(orbit_reference_frame);
    vessel.auto_pilot().set_target_direction(retrograde_direction.to_tuple());
    vessel.auto_pilot().engage();
    KSP::sleep_seconds(5);

    /* Burn retrograde and wait for periapsis to drop to target. */
    vessel.control().set_throttle(1.0);
    periapsis_event.acquire();
    periapsis_event.wait();
    periapsis_event.release();

    /* Cut throttle and decouple service module. */
    vessel.control().set_throttle(0.0);
    KSP::sleep_seconds(1);
    vessel.auto_pilot().set_target_direction(normal_direction.to_tuple());
    KSP::sleep_seconds(5);
    vessel.control().activate_next_stage();
    KSP::sleep_seconds(1);

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