#include "../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto body = vessel.orbit().body();

    /* Reference frames. */
    auto body_reference_frame = body.reference_frame();
    auto reference_frame = vessel.surface_reference_frame();

    /* Targets. */
    auto target_vector = KSP::Vector3(1, 0, 0);
    auto parachute_altitude = 1500;
    auto upper_atmosphere_altitude = body.flying_high_altitude_threshold();
    auto space_altitude = body.atmosphere_depth();

    /* Streams. */
    auto current_stage_stream = vessel.control().current_stage_stream();
    auto vertical_speed_stream = vessel.flight(body_reference_frame).vertical_speed_stream();
    auto altitude_stream = vessel.flight().mean_altitude_stream();

    /* Event for opening the parachute. */
    auto surface_altitude_call = vessel.flight(reference_frame).surface_altitude_call();
    auto parachute_event = connection.krpc.add_event(
        KSP::Expression::less_than_or_equal(
            connection.client,
            KSP::Expression::call(connection.client, surface_altitude_call),
            KSP::Expression::constant_double(connection.client, parachute_altitude)
        )
    );

    /* Resources. */
    std::unordered_map<int32_t, krpc::Stream<float>> resource_stream_map;
    resource_stream_map.insert(std::make_pair(5, vessel.resources_in_decouple_stage(4, false).amount_stream(KSP::resources::SOLID_FUEL)));
    resource_stream_map.insert(std::make_pair(4, vessel.resources_in_decouple_stage(3, false).amount_stream(KSP::resources::SOLID_FUEL)));
    resource_stream_map.insert(std::make_pair(3, vessel.resources_in_decouple_stage(2, false).amount_stream(KSP::resources::SOLID_FUEL)));
    resource_stream_map.insert(std::make_pair(2, vessel.resources_in_decouple_stage(1, false).amount_stream(KSP::resources::SOLID_FUEL)));

    /* Set auto pilot variables. */
    vessel.auto_pilot().set_reference_frame(reference_frame);
    vessel.auto_pilot().set_target_direction(target_vector.to_tuple());
    vessel.auto_pilot().engage();

    /* Countdown to launch. */
    KSP::countdown_seconds(3, "LAUNCH IN", "LAUNCH");

    /* Activate first stage. */
    vessel.control().activate_next_stage();
    KSP::sleep_seconds(1);

    /* Loop until apogee reached. */
    while (vertical_speed_stream() > 0)
    {
        auto current_altitude = altitude_stream();
        auto current_stage = current_stage_stream();

        /* Stage when fuel is low. */
        if (resource_stream_map.find(current_stage) != resource_stream_map.end()
            && resource_stream_map[current_stage]() < 0.1)
        {
            vessel.control().activate_next_stage();
        }

        /* Conduct science while flying high. */
        if (current_altitude > upper_atmosphere_altitude)
        {
            vessel.control().set_action_group(1, true);
        }

        /* Conduct science while flying low. */
        if (current_altitude > space_altitude)
        {
            vessel.control().set_action_group(2, true);
        }

        KSP::sleep_milliseconds(20);
    }

    /* Wait until parachute deploy. */
    parachute_event.acquire();
    parachute_event.wait();
    parachute_event.release();

    /* Open parachute. */
    vessel.control().activate_next_stage();
}