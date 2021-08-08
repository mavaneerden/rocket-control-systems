#include "../../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto target_vessel = connection.space_center.target_vessel();

    auto reference_frame = vessel.orbital_reference_frame();
    auto target_reference_frame = target_vessel.orbital_reference_frame();

    auto velocity_stream = vessel.velocity_stream(target_reference_frame);
    auto relative_velocity_stream = vessel.velocity_stream(target_reference_frame);
    auto relative_speed_stream = vessel.flight(target_reference_frame).speed_stream();
    auto relative_position_stream = vessel.position_stream(target_reference_frame);

    while(KSP::Vector3(relative_position_stream()).length() > 100)
    {
        vessel.auto_pilot().set_reference_frame(reference_frame);
        vessel.auto_pilot().set_target_direction((KSP::Vector3(relative_position_stream()) * -1).to_tuple());
        vessel.auto_pilot().engage();
        KSP::sleep_seconds(3);

        vessel.control().set_throttle(0.1);

        while (relative_speed_stream() < 5.0)
        {
            vessel.auto_pilot().set_target_direction((KSP::Vector3(relative_position_stream()) * -1).to_tuple());
        }

        vessel.control().set_throttle(0);

        auto throttle = 0.05;
        auto orbit = vessel.orbit();
        auto target_orbit = target_vessel.orbit();
        auto t_approach = orbit.time_of_closest_approach(target_orbit);
        auto ut_stream = connection.space_center.ut_stream();
        auto burn_time = KSP::get_burn_time(vessel, relative_speed_stream(), throttle);
        auto burn_start = t_approach - burn_time / 2;
        auto burn_stop = burn_start + burn_time;

        connection.space_center.warp_to(burn_start);

        while (ut_stream() <= burn_start)
        {
            vessel.auto_pilot().set_target_direction((KSP::Vector3(relative_velocity_stream()) * -1).to_tuple());
            burn_time = KSP::get_burn_time(vessel, relative_speed_stream(), throttle);
            burn_start = t_approach - burn_time / 2;
            burn_stop = burn_start + burn_time;

            KSP::sleep_milliseconds(100);
        }

        vessel.control().set_throttle(throttle);

        while (ut_stream() <= burn_stop)
        {
            vessel.auto_pilot().set_target_direction((KSP::Vector3(relative_velocity_stream()) * -1).to_tuple());
            KSP::sleep_milliseconds(10);
        }

        vessel.control().set_throttle(0);
        vessel.auto_pilot().set_target_direction((KSP::Vector3(relative_position_stream()) * -1).to_tuple());
        KSP::sleep_seconds(3);
    }
}

