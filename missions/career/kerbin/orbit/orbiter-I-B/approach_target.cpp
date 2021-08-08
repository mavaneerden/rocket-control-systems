#include "../../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto target_vessel = connection.space_center.target_vessel();

    auto reference_frame = vessel.orbital_reference_frame();

    auto velocity_stream = vessel.velocity_stream(reference_frame);
    auto target_velocity_stream = target_vessel.velocity_stream(reference_frame);
    auto relative_velocity = KSP::Vector3(target_velocity_stream()) - KSP::Vector3(velocity_stream());

    auto target_position_stream = target_vessel.position_stream(reference_frame);

    /* Burn to 0m/s relative to target. */
    vessel.auto_pilot().set_reference_frame(reference_frame);
    vessel.auto_pilot().set_target_direction(relative_velocity.to_tuple());
    vessel.auto_pilot().engage();
    KSP::sleep_seconds(5);

    while (relative_velocity.length() > 0.02)
    {
        auto throttle = relative_velocity.length() * 0.05;

        std::cout << relative_velocity.length() << std::endl;

        vessel.control().set_throttle(throttle);
        vessel.auto_pilot().set_target_direction(relative_velocity.to_tuple());

        relative_velocity = KSP::Vector3(target_velocity_stream()) - KSP::Vector3(velocity_stream());
    }

    vessel.control().set_throttle(0);
    KSP::sleep_milliseconds(100);
    vessel.auto_pilot().set_target_direction(target_position_stream());
    KSP::sleep_seconds(5);

    vessel.control().set_throttle(0.1);

    while (relative_velocity.length() < 5.0)
    {
        vessel.auto_pilot().set_target_direction(target_position_stream());
        relative_velocity = KSP::Vector3(target_velocity_stream()) - KSP::Vector3(velocity_stream());
    }

    vessel.control().set_throttle(0);

    auto closest_approach = vessel.orbit().distance_at_closest_approach(target_vessel.orbit());

    while (KSP::Vector3(target_position_stream()).length() > closest_approach + 5)
    {
        vessel.auto_pilot().set_target_direction(relative_velocity.to_tuple());
        relative_velocity = KSP::Vector3(target_velocity_stream()) - KSP::Vector3(velocity_stream());
    }

    vessel.auto_pilot().set_target_direction(relative_velocity.to_tuple());
    while (relative_velocity.length() > 0.02)
    {
        auto throttle = relative_velocity.length() * 0.05;

        std::cout << relative_velocity.length() << std::endl;

        vessel.control().set_throttle(throttle);
        vessel.auto_pilot().set_target_direction(relative_velocity.to_tuple());

        relative_velocity = KSP::Vector3(target_velocity_stream()) - KSP::Vector3(velocity_stream());
    }

    vessel.control().set_throttle(0);
}

