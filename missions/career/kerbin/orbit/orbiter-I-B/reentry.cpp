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
    auto radial_direction = KSP::Vector3(1, 0, 0);

    /* Target radial direction. */
    vessel.auto_pilot().set_reference_frame(orbit_reference_frame);
    vessel.auto_pilot().set_target_direction(radial_direction.to_tuple());
    vessel.auto_pilot().engage();
    KSP::sleep_seconds(10);
    vessel.auto_pilot().disengage();

    /* Undock vessels. */
    std::cout << "Undock the vessels!" << std::endl;
    KSP::wait_for_user();
    KSP::sleep_seconds(5);

    auto vessel1 = KSP::get_vessel_by_name("Orbiter I-B", connection);
    auto vessel2 = KSP::get_vessel_by_name("Orbiter I-B Target", connection);

    auto periapsis_altitude_stream1 = vessel1.orbit().periapsis_altitude_stream();
    auto periapsis_altitude_stream2 = vessel2.orbit().periapsis_altitude_stream();

    auto surface_altitude_stream1 = vessel1.flight(vessel1.surface_reference_frame()).surface_altitude_stream();
    auto surface_altitude_stream2 = vessel2.flight(vessel2.surface_reference_frame()).surface_altitude_stream();

    /* Target retrograde. */
    vessel1.auto_pilot().set_reference_frame(vessel1.orbital_reference_frame());
    vessel2.auto_pilot().set_reference_frame(vessel2.orbital_reference_frame());
    vessel1.auto_pilot().set_target_direction(retrograde_direction.to_tuple());
    vessel2.auto_pilot().set_target_direction(retrograde_direction.to_tuple());
    vessel1.auto_pilot().engage();
    vessel2.auto_pilot().engage();
    KSP::sleep_seconds(5);

    /* Burn retrograde and wait for periapsis to drop to target. */
    vessel1.control().set_throttle(1.0);
    vessel2.control().set_throttle(1.0);

    auto vessel1_bool = true;
    auto vessel2_bool = true;

    while (vessel1_bool || vessel2_bool)
    {
        if (periapsis_altitude_stream1() < periapsis_target)
        {
            vessel1.control().toggle_action_group(1);
            vessel1_bool = false;
        }
        if (periapsis_altitude_stream2() < periapsis_target)
        {
            vessel2.control().toggle_action_group(1);
            vessel2_bool = false;
        }

        std::cout << periapsis_altitude_stream1() << std::endl;
        std::cout << periapsis_altitude_stream2() << std::endl;

        KSP::sleep_milliseconds(20);
    }

    vessel1_bool = true;
    vessel2_bool = true;

    /* Cut throttle and decouple service module. */
    KSP::sleep_seconds(1);
    vessel1.auto_pilot().set_target_direction(normal_direction.to_tuple());
    vessel2.auto_pilot().set_target_direction(normal_direction.to_tuple());
    KSP::sleep_seconds(5);
    vessel1.control().set_action_group(2, true);
    vessel2.control().set_action_group(2, true);
    KSP::sleep_seconds(1);

    /* Orient spacecraft towards surface retrograde. */
    vessel1.auto_pilot().set_reference_frame(surface_velocity_reference_frame);
    vessel1.auto_pilot().set_target_direction(retrograde_direction.to_tuple());
    vessel2.auto_pilot().set_reference_frame(surface_velocity_reference_frame);
    vessel2.auto_pilot().set_target_direction(retrograde_direction.to_tuple());

    /* Wait until parachute deploy. */
    while (vessel1_bool || vessel2_bool)
    {
        if (surface_altitude_stream1() < parachute_altitude)
        {
            vessel1.control().toggle_action_group(3);
            vessel1_bool = false;
        }
        if (surface_altitude_stream2() < parachute_altitude)
        {
            vessel2.control().toggle_action_group(3);
            vessel2_bool = false;
        }

        KSP::sleep_milliseconds(20);
    }
}