#pragma once

#include <math.h>
#include "enums/types.hpp"
#include "connection.hpp"
#include "node_executor.hpp"
#include "sleep.hpp"
#include "vector3.hpp"

namespace KSP
{
    class Maneuver
    {
    private:
        Connection m_connection;
        Vessel m_vessel;
    public:
        Maneuver(Connection connection, Vessel vessel);
        ~Maneuver();
    public:
        void cicularize(bool raise_orbit);
        void change_inclination(Body target);
        void lower_orbit_from_apoapsis(double periapsis_target);
        void raise_orbit_from_periapsis(double apoapsis_target);
        void transfer_to_body(Body target);
    private:
        double calculate_velocity(Orbit orbit);
        double calculate_velocity(Orbit orbit, double apoapsis, double periapsis, double altitude);
        double calculate_inclination_change_delta_v(double orbital_velocity, double inclination_change);
        double calculate_intercept_angle(double current_radius, double target_radius, float gravitational_parameter);
        double calculate_intercept_angle(Orbit current_orbit, Orbit target_orbit);
        double calculate_transfer_time(double current_radius, double target_radius, float gravitational_parameter);
        double calculate_transfer_delta_v(double current_radius, double target_radius, double gravitational_parameter);
        double calculate_transfer_delta_v(Orbit current_orbit, Orbit target_orbit);
    };

    Maneuver::Maneuver(Connection connection, Vessel vessel) : m_connection(connection), m_vessel(vessel)
    {
    }

    Maneuver::~Maneuver()
    {
    }

    void Maneuver::cicularize(bool raise_orbit)
    {
        auto orbit = m_vessel.orbit();
        auto apoapsis_altitude = orbit.apoapsis_altitude();
        auto periapsis_altitude = orbit.periapsis_altitude();
        auto target_altitude = raise_orbit ? apoapsis_altitude : periapsis_altitude;
        auto target_time = m_connection.space_center.ut() + (raise_orbit ? orbit.time_to_apoapsis() : orbit.time_to_periapsis());
        auto current_velocity = calculate_velocity(orbit, apoapsis_altitude, periapsis_altitude, target_altitude);
        auto target_velocity = calculate_velocity(orbit, target_altitude, target_altitude, target_altitude);
        auto required_delta_v = target_velocity - current_velocity;
        auto maneuver_node = m_vessel.control().add_node(target_time, required_delta_v);

        NodeExecutor executor(maneuver_node, m_vessel);
        executor.execute(m_connection, 1.0);
    }

    void Maneuver::change_inclination(Body target)
    {
        auto vessel_orbit = m_vessel.orbit();
        auto reference_frame = vessel_orbit.body().non_rotating_reference_frame();

        auto vessel_position = Vector3(m_vessel.position(reference_frame));
        auto vessel_velocity = Vector3(m_vessel.velocity(reference_frame));
        auto target_position = Vector3(target.position(reference_frame));
        auto target_velocity = Vector3(target.velocity(reference_frame));
        auto vessel_inclination = vessel_orbit.inclination();
        auto target_inclination = target.orbit().inclination();
        auto vessel_lan = vessel_orbit.longitude_of_ascending_node();
        auto target_lan = target.orbit().longitude_of_ascending_node();
        auto inclination_change = vessel_orbit.relative_inclination(target.orbit());

        auto vessel_orbital_speed = calculate_velocity(vessel_orbit);
        auto vessel_angular_speed = vessel_orbital_speed / vessel_orbit.semi_major_axis();

        auto vessel_normal = vessel_velocity.cross(vessel_position).normalize();
        auto target_normal = target_velocity.cross(target_position).normalize();
        auto an_dn_vector = target_normal.cross(vessel_normal).normalize();
        auto vessel_angle = vessel_position.angle_2d(an_dn_vector);

        double time_to_node;

        if (vessel_angle < 0)
        {
            /* Target ascending node at pi rad. */
            time_to_node = (vessel_angle + M_PI) / vessel_angular_speed;
        }
        else
        {
            /* Target descending node at 0 rad. */
            time_to_node = vessel_angle / vessel_angular_speed;
            inclination_change *= -1;
        }

        auto delta_v = calculate_inclination_change_delta_v(vessel_orbital_speed, inclination_change);
        auto delta_v_prograde = delta_v * sin(inclination_change / 2);
        auto delta_v_normal = delta_v * cos(inclination_change / 2);
        auto maneuver_node = m_vessel.control().add_node(m_connection.space_center.ut() + time_to_node, -abs(delta_v_prograde), -delta_v_normal);

        NodeExecutor executor(maneuver_node, m_vessel);
        executor.execute(m_connection, 1.0);
    }

    void Maneuver::lower_orbit_from_apoapsis(double periapsis_target)
    {
        auto orbit = m_vessel.orbit();
        auto apoapsis_altitude = orbit.apoapsis_altitude();
        auto periapsis_altitude = orbit.periapsis_altitude();
        auto target_altitude = periapsis_target;
        auto current_velocity = calculate_velocity(orbit, apoapsis_altitude, periapsis_altitude, apoapsis_altitude);
        auto target_velocity = calculate_velocity(orbit, apoapsis_altitude, target_altitude, apoapsis_altitude);
        auto required_delta_v = target_velocity - current_velocity;
        auto maneuver_node = m_vessel.control().add_node(m_connection.space_center.ut() + orbit.time_to_apoapsis(), required_delta_v);

        NodeExecutor executor(maneuver_node, m_vessel);
        executor.execute(m_connection, 1.0);
    }

    void Maneuver::raise_orbit_from_periapsis(double apoapsis_target)
    {
        auto orbit = m_vessel.orbit();
        auto apoapsis_altitude = orbit.apoapsis_altitude();
        auto periapsis_altitude = orbit.periapsis_altitude();
        auto target_altitude = apoapsis_target;
        auto current_velocity = calculate_velocity(orbit, apoapsis_altitude, periapsis_altitude, periapsis_altitude);
        auto target_velocity = calculate_velocity(orbit, target_altitude, periapsis_altitude, periapsis_altitude);
        auto required_delta_v = target_velocity - current_velocity;
        auto maneuver_node = m_vessel.control().add_node(m_connection.space_center.ut() + orbit.time_to_periapsis(), required_delta_v);

        NodeExecutor executor(maneuver_node, m_vessel);
        executor.execute(m_connection, 1.0);
    }

    void Maneuver::transfer_to_body(Body target)
    {
        auto current_orbit = m_vessel.orbit();
        auto target_orbit = target.orbit();
        auto reference_frame = m_vessel.orbit().body().non_rotating_reference_frame();
        auto target_position = Vector3(target.position(reference_frame));
        auto vessel_position = Vector3(m_vessel.position(reference_frame));
        auto target_angle = calculate_intercept_angle(current_orbit, target_orbit);
        auto current_angular_velocity = 2 * M_PI / current_orbit.period();
        auto target_angular_velocity = 2 * M_PI / target_orbit.period();
        auto current_angle = vessel_position.angle_2d(target_position);
        double angle_difference;
        double time_to_transfer;

        if (current_angle < 0)
        {
            angle_difference = target_angle - current_angle;
            angle_difference = angle_difference < 0 ? angle_difference + 2 * M_PI : angle_difference;
            time_to_transfer = angle_difference / (target_angular_velocity - current_angular_velocity);
        }
        else
        {
            angle_difference = current_angle - target_angle;
            angle_difference = angle_difference < 0 ? angle_difference + 2 * M_PI : angle_difference;
            time_to_transfer = angle_difference / (current_angular_velocity - target_angular_velocity);
        }

        time_to_transfer = time_to_transfer < 0 ? time_to_transfer + current_orbit.period() : time_to_transfer;

        auto delta_v_required = calculate_transfer_delta_v(current_orbit, target_orbit);
        auto maneuver_node = m_vessel.control().add_node(m_connection.space_center.ut() + time_to_transfer, delta_v_required);

        NodeExecutor executor(maneuver_node, m_vessel);
        executor.execute(m_connection, 1.0);
    }

    double Maneuver::calculate_velocity(Orbit orbit)
    {
        auto body = orbit.body();
        auto mu = body.gravitational_parameter();
        auto sma = orbit.semi_major_axis();

        return sqrt(mu * (1 / sma));
    }

    double Maneuver::calculate_velocity(Orbit orbit, double apoapsis, double periapsis, double altitude)
    {
        auto body = orbit.body();
        auto mu = body.gravitational_parameter();
        auto radius = body.equatorial_radius();
        auto sma = (apoapsis + periapsis) / 2 + radius;

        return sqrt(mu * (2 / (radius + altitude) - 1 / sma));
    }

    double Maneuver::calculate_inclination_change_delta_v(double orbital_velocity, double inclination_change)
    {
        return 2 * orbital_velocity * sin(inclination_change / 2);
    }

    double Maneuver::calculate_intercept_angle(double current_radius, double target_radius, float gravitational_parameter)
    {
        return M_PI - sqrt(gravitational_parameter / target_radius) * (calculate_transfer_time(current_radius, target_radius, gravitational_parameter) / target_radius);
    }

    double Maneuver::calculate_intercept_angle(Orbit current_orbit, Orbit target_orbit)
    {
        double current_radius = current_orbit.semi_major_axis();
        double target_radius = target_orbit.semi_major_axis();
        float gravitational_parameter = current_orbit.body().gravitational_parameter();

        return calculate_intercept_angle(current_radius, target_radius, gravitational_parameter);
    }

    double Maneuver::calculate_transfer_time(double current_radius, double target_radius, float gravitational_parameter)
    {
        return M_PI * sqrt(pow(current_radius + target_radius, 3) / (8 * gravitational_parameter));
    }

    double Maneuver::calculate_transfer_delta_v(double current_radius, double target_radius, double gravitational_parameter)
    {
        return sqrt(gravitational_parameter / current_radius) * (sqrt(2 * target_radius / (current_radius + target_radius)) - 1);
    }

    double Maneuver::calculate_transfer_delta_v(Orbit current_orbit, Orbit target_orbit)
    {
        double current_radius = current_orbit.semi_major_axis();
        double target_radius = target_orbit.semi_major_axis();
        float gravitational_parameter = current_orbit.body().gravitational_parameter();

        return calculate_transfer_delta_v(current_radius, target_radius, gravitational_parameter);
    }
}
