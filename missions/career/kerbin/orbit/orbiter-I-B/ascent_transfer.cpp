#include "../../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto body = vessel.orbit().body();

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

    /* Streams. */
    auto current_stage_stream = vessel.control().current_stage_stream();
    auto vertical_speed_stream = vessel.flight(body_reference_frame).vertical_speed_stream();
    auto altitude_stream = vessel.flight().mean_altitude_stream();
    auto apoapsis_stream = vessel.orbit().apoapsis_altitude_stream();

    /* Event for being out of the atmosphere. */
    auto altitude_call = vessel.flight().mean_altitude_call();
    auto out_of_atmosphere_event = connection.krpc.add_event(
        KSP::Expression::greater_than(
            connection.client,
            KSP::Expression::call(connection.client, altitude_call),
            KSP::Expression::constant_double(connection.client, space_altitude)
        )
    );

    /* Resources. */
    KSP::ResourcesMap resources;
    resources.insert(std::make_pair(5, vessel.resources_in_decouple_stage(4, false).amount_stream(KSP::resources::LIQUID_FUEL)));

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

        /* Use Science Jr. while flying high. */
        if (current_altitude > upper_atmosphere_altitude)
        {
            vessel.control().set_action_group(1, true);
        }

        KSP::sleep_milliseconds(20);
    }

    /* Cut throttle and coast until out of atmosphere. */
    vessel.control().set_throttle(0.0);
    out_of_atmosphere_event.acquire();
    out_of_atmosphere_event.wait();
    out_of_atmosphere_event.release();
    KSP::sleep_seconds(2);

    /* Jettison nose cone. */
    vessel.control().activate_next_stage();

    /* Create and execute circularisation maneuver node. */
    auto maneuver = KSP::Maneuver(connection, vessel);
    maneuver.cicularize(true);
}