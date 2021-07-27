#include "../../lib/ksp.hpp"

void launch(KSP::Connection connection)
{
    /* Get the active vessel. */
    auto vessel = connection.space_center.active_vessel();
    /* Reference frame used throughout the flight. */
    auto reference_frame = vessel.surface_reference_frame();
    auto surface_reference_frame = vessel.surface_reference_frame();

    /* Target values. */
    auto target_direction = KSP::Vector3(1, 0, 0);
    auto target_altitude = 80000;
    auto separation_altitude = 55000;
    auto target_twr_max = 3.0;
    /* Body values. */
    auto body = vessel.orbit().body();
    auto gravitational_paramter = body.gravitational_parameter();
    auto body_radius = body.equatorial_radius();
    auto body_reference_frame = body.reference_frame();

    /* Streams. */
    auto apoapsis_altitude_stream = vessel.orbit().apoapsis_altitude_stream();
    auto altitude_stream = vessel.flight().mean_altitude_stream();
    auto thrust_stream = vessel.thrust_stream();
    auto mass_stream = vessel.mass_stream();
    auto available_thrust_stream = vessel.available_thrust_stream();
    auto surface_velocity_stream = vessel.velocity_stream(body_reference_frame);

    /* Set correct reference frame for the auto pilot. */
    vessel.auto_pilot().set_reference_frame(reference_frame);
    /* Target the 'up' direction. */
    vessel.auto_pilot().set_target_direction(target_direction.to_tuple());
    /* Enable auto pilot. */
    vessel.auto_pilot().engage();

    /* Launch countdown. */
    KSP::countdown_seconds(3, "LAUNCH IN", "LAUNCH");

    /* Activate the engines. */
    vessel.control().set_throttle(1.0);
    vessel.control().activate_next_stage();

    /* Wait. */
    KSP::sleep_seconds(3);

    /* Release launch clamps. */
    vessel.control().activate_next_stage();

    /* Maintain maximum thrust-to-weight ratio throughout flight until target apoapsis is reached. */
    while (apoapsis_altitude_stream() < target_altitude)
    {
        auto g = gravitational_paramter / pow(altitude_stream() + body_radius, 2);
        auto twr_current = thrust_stream() / (mass_stream() * g);
        auto thrust_target = target_twr_max * mass_stream() * g;
        auto throttle = thrust_target / available_thrust_stream();

        vessel.control().set_throttle(throttle);

        auto surface_velocity = KSP::Vector3(connection.space_center.transform_direction(surface_velocity_stream(), body_reference_frame, surface_reference_frame));
        auto surface_speed = surface_velocity.length();
        auto horizontal_velocity = surface_velocity.projection_on_plane(target_direction);
        auto horizontal_speed = horizontal_velocity.length();
        auto vertical_velocity = surface_velocity.projection(target_direction);
        auto vertical_speed = vertical_velocity.length();

        std::cout << "Angle: " << (0.5 / (-5 * horizontal_speed - 0.5) + 1) << std::endl;
        auto new_target_length = vertical_speed / cos(acos(vertical_speed / surface_speed) + (0.5 / (-5 * horizontal_speed - 0.5) + 1) * (M_PI / 180));
        auto new_target_horizontal_factor = sqrt(pow(new_target_length, 2) - pow(vertical_speed, 2)) / horizontal_speed;
        auto new_target = surface_velocity - (horizontal_velocity + horizontal_velocity * new_target_horizontal_factor);

        vessel.auto_pilot().set_target_direction(new_target.to_tuple());

        std::cout << "VELHOR:  " << horizontal_velocity << std::endl;
        std::cout << "VELVER:  " << vertical_velocity << std::endl;
        std::cout << "SPDSURF: " << surface_velocity.length() << std::endl;
        std::cout << "TARLEN:  " << new_target_length << std::endl;
        std::cout << "SPDHOR:  " << horizontal_velocity.length() << std::endl;
        std::cout << "SPDVER:  " << vertical_velocity.length() << std::endl;
        std::cout << "VELSURF: " << surface_velocity << std::endl;
        std::cout << "TARGET:  " << new_target << std::endl;

        KSP::sleep_milliseconds(20);
    }

    /* Cut the engines. */
    vessel.control().set_throttle(0.0);

    /* Wait until an altitude of 55km is reached. */
    while (altitude_stream() < separation_altitude)
    {
        KSP::sleep_milliseconds(100);
    }

    /* Separate the two craft. */
    vessel.control().activate_next_stage();

    /* Get the booster vessel. */
    auto booster_vessel = KSP::get_vessel_by_name("New Shepard", connection);

    /* Event that triggers when the booster is out of the atmosphere. */
    auto altitude_call = vessel.flight().mean_altitude_call();
    auto out_of_atmosphere_event = connection.krpc.add_event(
        KSP::Expression::greater_than(
            connection.client,
            KSP::Expression::call(connection.client, altitude_call),
            KSP::Expression::constant_double(connection.client, body.atmosphere_depth())
        )
    );

    /* Set correct attitude. */
    booster_vessel.auto_pilot().set_reference_frame(reference_frame);
    booster_vessel.auto_pilot().set_target_direction(target_direction.to_tuple());
    booster_vessel.auto_pilot().engage();

    /* Wait until booster is out of the atmosphere. */
    out_of_atmosphere_event.acquire();
    out_of_atmosphere_event.wait();
    out_of_atmosphere_event.release();
}