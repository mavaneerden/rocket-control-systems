#include "../../../../lib/ksp.hpp"

int main(int argc, char const *argv[])
{
    /* Automatically connects to the server with the given IP address. */
    auto connection = KSP::Connection();
    auto vessel = connection.space_center.active_vessel();
    auto target_vessel = connection.space_center.target_vessel();

    auto orbit = vessel.orbit();
    auto target_orbit = target_vessel.orbit();
    auto body = orbit.body();
    auto body_reference_frame = body.non_rotating_reference_frame();
    auto vessel_reference_frame = vessel.orbital_reference_frame();
    auto orbit_reference_frame = body_reference_frame.create_hybrid(connection.client, body_reference_frame, vessel_reference_frame);

    auto t_approach = orbit.time_of_closest_approach(target_orbit);
    auto position = orbit.position_at(t_approach, body_reference_frame);
    auto eccentric_anomaly_difference = abs(orbit.eccentric_anomaly() - orbit.eccentric_anomaly_at_ut(t_approach));
    auto velocity = KSP::velocity_at(orbit, t_approach);
    auto velocity_target = KSP::velocity_at(target_orbit, t_approach);

    /**
     * Rotate the vessel orbital reference frame around the normal direction so
     * the prograde/radial vectors are correct.
     * Normal vector stays the same because inclination does not change.
     */
    auto conversion_reference_frame = krpc::services::SpaceCenter::ReferenceFrame::create_relative(
        connection.client,
        vessel_reference_frame,
        {0.0, 0.0, 0.0},
        {0.0, 0.0, -eccentric_anomaly_difference, 1.0}
    );
    auto relative_retrograde = KSP::Vector3(connection.space_center.transform_direction((velocity_target - velocity).to_tuple(), body_reference_frame, conversion_reference_frame));
    // auto node = vessel.control().add_node(t_approach, relative_retrograde.m_y, relative_retrograde.m_z, relative_retrograde.m_x);
    // auto executor = KSP::NodeExecutor(node, vessel);


    // std::cout << relative_retrograde << std::endl;
    // std::cout << velocity << std::endl;
    // std::cout << velocity_target << std::endl;
    // std::cout << relative_retrograde.length() << std::endl;

    // executor.execute(connection, 1.0);
    // auto target_reference_frame_stream = target_vessel.orbital_reference_frame_stream();
    // auto relative_velocity_stream = vessel.velocity(target_reference_frame_stream());

    auto throttle = 0.3;
    auto relative_speed = relative_retrograde.length();
    auto burn_time = KSP::get_burn_time(vessel, relative_speed, throttle);
    auto lead_burn_time = KSP::get_burn_time(vessel, relative_speed, throttle, 0.5);
    auto ut_stream = connection.space_center.ut_stream();
    auto burn_start = t_approach - lead_burn_time;
    auto burn_stop = burn_start + burn_time;

    connection.space_center.warp_to(burn_start - 10);
    vessel.auto_pilot().set_reference_frame(vessel_reference_frame);
    vessel.auto_pilot().engage();

    while (ut_stream() <= burn_start)
    {
        vessel.auto_pilot().set_target_direction((KSP::Vector3(vessel.velocity(target_vessel.orbital_reference_frame())) * -1).to_tuple());
        KSP::sleep_milliseconds(10);
    }

    vessel.control().set_throttle(throttle);

    while (ut_stream() <= burn_stop)
    {
        vessel.auto_pilot().set_target_direction((KSP::Vector3(vessel.velocity(target_vessel.orbital_reference_frame())) * -1).to_tuple());
        KSP::sleep_milliseconds(10);
    }

    vessel.control().set_throttle(0);

    throttle = 0.05;
    relative_speed = vessel.flight(target_vessel.orbital_reference_frame()).speed();
    burn_time = KSP::get_burn_time(vessel, relative_speed, throttle);
    lead_burn_time = KSP::get_burn_time(vessel, relative_speed, throttle, 0.5);
    ut_stream = connection.space_center.ut_stream();

    vessel.auto_pilot().set_target_direction((KSP::Vector3(vessel.velocity(target_vessel.orbital_reference_frame())) * -1).to_tuple());
    KSP::sleep_seconds(5);

    burn_start = ut_stream();
    burn_stop = burn_start + burn_time;

    vessel.control().set_throttle(throttle);

    while (ut_stream() <= burn_stop)
    {
        vessel.auto_pilot().set_target_direction((KSP::Vector3(vessel.velocity(target_vessel.orbital_reference_frame())) * -1).to_tuple());
        KSP::sleep_milliseconds(10);
    }

    vessel.control().set_throttle(0);
}
