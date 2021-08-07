#include "../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto body = vessel.orbit().body();
    auto mun = connection.get_body(KSP::bodies::MUN);

    /* Reference frames. */
    auto body_reference_frame = body.reference_frame();
    auto surface_reference_frame = vessel.surface_reference_frame();
    auto orbit_reference_frame = vessel.orbital_reference_frame();

    /* Targets. */
    auto parachute_altitude = 1500;
    auto upper_atmosphere_altitude = body.flying_high_altitude_threshold();
    auto space_altitude = body.atmosphere_depth();
    auto target_apoapsis = space_altitude + 5000;
    auto prograde_direction = KSP::Vector3(0, 1, 0);
    auto periapsis_target = 15000;
    auto retrograde_direction = KSP::Vector3(0, -1, 0);

    /* Streams. */
    auto current_stage_stream = vessel.control().current_stage_stream();
    auto vertical_speed_stream = vessel.flight(body_reference_frame).vertical_speed_stream();
    auto altitude_stream = vessel.flight().mean_altitude_stream();
    auto apoapsis_stream = vessel.orbit().apoapsis_altitude_stream();

    /* Event for opening the parachute. */
    auto surface_altitude_call = vessel.flight(surface_reference_frame).surface_altitude_call();
    auto parachute_event = connection.krpc.add_event(
        KSP::Expression::less_than_or_equal(
            connection.client,
            KSP::Expression::call(connection.client, surface_altitude_call),
            KSP::Expression::constant_double(connection.client, parachute_altitude)
        )
    );

    /* Event for being out of the atmosphere. */
    auto altitude_call = vessel.flight().mean_altitude_call();
    auto out_of_atmosphere_event = connection.krpc.add_event(
        KSP::Expression::greater_than(
            connection.client,
            KSP::Expression::call(connection.client, altitude_call),
            KSP::Expression::constant_double(connection.client, space_altitude)
        )
    );

    /* Event for being near apoapsis. */
    auto time_to_apoapsis_call = vessel.orbit().time_to_apoapsis_call();
    auto apoapsis_reached_event = connection.krpc.add_event(
        KSP::Expression::less_than(
            connection.client,
            KSP::Expression::call(connection.client, time_to_apoapsis_call),
            KSP::Expression::constant_double(connection.client, 8.0)
        )
    );

    /* Event for circularisation burn. */
    auto periapsis_call = vessel.orbit().periapsis_altitude_call();
    auto apoapsis_call = vessel.orbit().periapsis_altitude_call();
    auto circularisation_event = connection.krpc.add_event(
        KSP::Expression::and_(
            connection.client,
            KSP::Expression::greater_than(
                connection.client,
                KSP::Expression::call(connection.client, periapsis_call),
                KSP::Expression::constant_double(connection.client, space_altitude)
            ),
            KSP::Expression::greater_than(
                connection.client,
                KSP::Expression::call(connection.client, apoapsis_call),
                KSP::Expression::constant_double(connection.client, space_altitude)
            )
        )
    );

    /* Resources. */
    KSP::ResourcesMap resources;
    resources.insert(std::make_pair(4, vessel.resources_in_decouple_stage(3, false).amount_stream(KSP::resources::SOLID_FUEL)));
    resources.insert(std::make_pair(3, vessel.resources_in_decouple_stage(2, false).amount_stream(KSP::resources::LIQUID_FUEL)));

    /* Create launcher. */
    KSP::Launcher launcher(vessel, resources);

    /* Set auto pilot variables. */
    vessel.auto_pilot().target_pitch_and_heading(90, 90);
    vessel.auto_pilot().engage();

    /* Launch the vessel. */
    launcher.launch(1.0, 3);

    /* Loop until apogee reached. */
    while (apoapsis_stream() < target_apoapsis)
    {
        auto current_altitude = altitude_stream();

        launcher.step(current_stage_stream(), current_altitude, vertical_speed_stream());

        KSP::sleep_milliseconds(20);
    }

    /* Cut throttle and coast until out of atmosphere. */
    vessel.control().set_throttle(0.0);
    out_of_atmosphere_event.acquire();
    out_of_atmosphere_event.wait();
    out_of_atmosphere_event.release();
    KSP::sleep_seconds(2);

    /* Create and execute circularisation maneuver node. */
    auto maneuver = KSP::Maneuver(connection, vessel);
    maneuver.cicularize(true);
    KSP::sleep_seconds(1);

    /* Change inclination to match the Mun's. */
    maneuver.change_inclination(mun);
    KSP::sleep_seconds(1);

    /* Transfer to the Mun. */
    maneuver.transfer_to_body(mun);
    KSP::sleep_seconds(1);

    /* Periapsis event for Mun. */
    auto periapsis_altitude_call = vessel.orbit().next_orbit().periapsis_altitude_call();
    auto periapsis_event = connection.krpc.add_event(
        KSP::Expression::greater_than(
            connection.client,
            KSP::Expression::call(connection.client, periapsis_altitude_call),
            KSP::Expression::constant_double(connection.client, periapsis_target)
        )
    );

    /* Target retrograde. */
    vessel.auto_pilot().set_reference_frame(orbit_reference_frame);
    vessel.auto_pilot().set_target_direction(retrograde_direction.to_tuple());
    vessel.auto_pilot().engage();
    KSP::sleep_seconds(5);

    /* Burn retrograde and wait for periapsis to drop to target. */
    vessel.control().set_throttle(0.1);
    periapsis_event.acquire();
    periapsis_event.wait();
    periapsis_event.release();
    vessel.control().set_throttle(0.0);
}