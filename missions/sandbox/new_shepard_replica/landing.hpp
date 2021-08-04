#include "../../lib/ksp.hpp"

void landing(KSP::Connection connection)
{
    /* Vessels. */
    auto booster_vessel = KSP::get_vessel_by_name("New Shepard", connection);
    auto capsule_vessel = KSP::get_vessel_by_name("New Shepard Capsule", connection);

    /* Reference frames. */
    auto booster_reference_frame_normal = booster_vessel.reference_frame();
    auto booster_reference_frame = booster_vessel.surface_reference_frame();
    auto capsule_reference_frame = capsule_vessel.surface_reference_frame();

    /* Altitude target values. */
    auto dragbrake_altitude = 18000;
    auto drogue_parachute_altitude = 2000;
    auto main_parachute_altitude = 1000;
    auto up_vector = KSP::Vector3(1, 0, 0);

    /* Hoverslam values. */
    auto hoverslam_pid = KSP::PID(connection, 0.015, 0.020, 0.0005);
    auto velocity_pid = KSP::PID(connection, 0.30, 0.02, 0.005);
    auto lander = KSP::Lander(connection);
    auto constant_speed_target = -1.0;
    auto target_throttle = 0.8;
    auto hoverslam_target = 4;
    auto ship_height = 8.6;

    /* Body values. */
    auto body = booster_vessel.orbit().body();
    auto gravitational_paramter = body.gravitational_parameter();
    auto body_radius = body.equatorial_radius();
    auto body_reference_frame = body.reference_frame();

    /* Streams. */
    auto booster_altitude_stream = booster_vessel.flight().mean_altitude_stream();
    auto booster_surface_altitude_stream = booster_vessel.flight(booster_reference_frame).surface_altitude_stream();
    auto booster_surface_speed_stream = booster_vessel.flight(body_reference_frame).speed_stream();
    auto booster_mass_stream = booster_vessel.mass_stream();
    auto booster_available_thrust_stream = booster_vessel.available_thrust_stream();
    auto booster_vertical_surface_speed_stream = booster_vessel.flight(body_reference_frame).vertical_speed_stream();
    auto booster_surface_velocity_stream = booster_vessel.velocity_stream(body_reference_frame);
    auto booster_drag_stream = booster_vessel.flight(booster_reference_frame).drag_stream();
    auto capsule_altitude_stream = capsule_vessel.flight(capsule_reference_frame).surface_altitude_stream();

    bool completed_stages[5] = {false};

    booster_vessel.auto_pilot().engage();

    while (capsule_vessel.situation() != KSP::Situation::landed)
    {
        /* Dragbrakes deployment event. */
        if (!completed_stages[0] && booster_altitude_stream() < dragbrake_altitude)
        {
            completed_stages[0] = true;
            booster_vessel.control().set_brakes(true);
        }

        /* Drogue parachute deployment event. */
        if (!completed_stages[1] && capsule_altitude_stream() < drogue_parachute_altitude)
        {
            completed_stages[1] = true;
            capsule_vessel.control().set_action_group(2, true);
        }

        /* Main parachute deployment event. */
        if (!completed_stages[2] && capsule_altitude_stream() < main_parachute_altitude)
        {
            completed_stages[2] = true;
            capsule_vessel.control().set_action_group(3, true);
        }

        if (completed_stages[0] && !completed_stages[3] && booster_vertical_surface_speed_stream() < -5)
        {
            /* Adjust throttle for hoverslam. */
            auto throttle = lander.vertical_hoverslam_throttle(
                hoverslam_pid,
                body,
                booster_mass_stream(),
                booster_altitude_stream(),
                booster_surface_altitude_stream(),
                booster_surface_speed_stream(),
                ship_height,
                hoverslam_target,
                booster_available_thrust_stream() * target_throttle,
                KSP::Vector3(booster_drag_stream()).m_x
            );

            booster_vessel.control().set_throttle(throttle);

            /* Target surface retrograde until above 10 m/s down. */
            if (booster_vertical_surface_speed_stream() < -15)
            {
                booster_vessel.auto_pilot().set_target_direction((KSP::Vector3(connection.space_center.transform_direction(booster_surface_velocity_stream(), body_reference_frame, booster_reference_frame)) * -1).to_tuple());
            }
            else
            {
                booster_vessel.control().set_gear(true);
                booster_vessel.auto_pilot().set_target_direction(up_vector.to_tuple());
            }
        }
        else if (completed_stages[0] && !completed_stages[3])
        {
            completed_stages[3] = true;
            booster_vessel.control().set_throttle(0.10);
            velocity_pid.start();
        }

        /* Go down with a constant velocity. */
        if (completed_stages[3] && !completed_stages[4] && booster_vessel.situation() != krpc::services::SpaceCenter::VesselSituation::landed)
        {
            auto vertical_speed = booster_vertical_surface_speed_stream();
            auto surface_altitude = booster_surface_altitude_stream();
            auto ship_up = KSP::Vector3(connection.space_center.transform_direction(KSP::Vector3(0, 1, 0).to_tuple(), booster_reference_frame_normal, booster_reference_frame));
            auto throttle_control = -(vertical_speed - constant_speed_target - 2.066) * KSP::get_g_at_altitude(body, booster_altitude_stream()) * booster_mass_stream() / (2 * booster_available_thrust_stream());

            auto horizontal_correction = cos(ship_up.angle_3d(up_vector));
            auto surface_velocity = KSP::Vector3(connection.space_center.transform_direction(booster_surface_velocity_stream(), body_reference_frame, booster_reference_frame));
            auto desired_velocity = KSP::Vector3(constant_speed_target, 0, 0);
            auto delta_velocity = desired_velocity - surface_velocity;
            auto target_vector = up_vector * KSP::get_g_at_altitude(body, booster_altitude_stream()) + delta_velocity;

            booster_vessel.auto_pilot().set_target_direction(target_vector.to_tuple());

            booster_vessel.control().set_throttle(throttle_control / horizontal_correction);

            std::cout << "CONTROL:        " << throttle_control << std::endl;
            std::cout << "HORIZONTAL:     " << horizontal_correction << std::endl;
            std::cout << "ANGLE:          " << ship_up.angle_3d(up_vector) << std::endl;
            std::cout << "Vertical speed: " << vertical_speed << std::endl;
        }
        else if (completed_stages[3] && !completed_stages[4])
        {
            completed_stages[4] = true;
            booster_vessel.control().set_throttle(0);
            booster_vessel.auto_pilot().disengage();
        }

        KSP::sleep_milliseconds(10);
    }
}