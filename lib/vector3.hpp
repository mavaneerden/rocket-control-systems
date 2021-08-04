#pragma once

#include <tuple>
#include <math.h>
#include <sstream>

namespace KSP
{
    class Vector3
    {
    public:
        double m_x;
        double m_y;
        double m_z;
    public:
        Vector3();
        Vector3(double x, double y, double z);
        Vector3(std::tuple<double, double, double> tuple);
        ~Vector3();
    public:
        std::tuple<double, double, double> to_tuple();
        Vector3 normalize();
        double length();
        double angle_2d(Vector3 vector);
        double angle_3d(Vector3 vector);
        double dot(Vector3 vector);
        Vector3 cross(Vector3 vector);
        Vector3 rotate(Vector3 axis, double angle);
        Vector3 projection(Vector3 vector);
        Vector3 projection_on_plane(Vector3 normal);
    public:
        friend Vector3 operator*(const Vector3& vector, double scalar);
        friend Vector3 operator/(const Vector3& vector, double scalar);
        friend Vector3 operator+(const Vector3& vector, double scalar);
        friend Vector3 operator-(const Vector3& vector, double scalar);
        friend Vector3 operator+(const Vector3& left, const Vector3& right);
        friend Vector3 operator-(const Vector3& left, const Vector3& right);
        friend std::ostream& operator<<(std::ostream& out, const Vector3& v);
    };

    Vector3::Vector3() : m_x(0), m_y(0), m_z(0)
    {
    }

    Vector3::Vector3(double x, double y, double z) : m_x(x), m_y(y), m_z(z)
    {
    }

    Vector3::Vector3(std::tuple<double, double, double> tuple) : m_x(std::get<0>(tuple)), m_y(std::get<1>(tuple)), m_z(std::get<2>(tuple))
    {
    }

    Vector3::~Vector3()
    {
    }

    std::tuple<double, double, double> Vector3::to_tuple()
    {
        return std::make_tuple(m_x, m_y, m_z);
    }

    Vector3 Vector3::normalize()
    {
        return *this / length();
    }

    double Vector3::length()
    {
        return sqrt(dot(*this));
    }

    double Vector3::angle_2d(Vector3 vector)
    {
        auto dot_product = m_x * vector.m_x + m_z * vector.m_z;
        auto determiniant = m_x * vector.m_z - m_z * vector.m_x;

        return atan2(determiniant, dot_product);
    }

    double Vector3::angle_3d(Vector3 vector)
    {
        return acos(this->normalize().dot(vector.normalize()));
    }

    double Vector3::dot(Vector3 vector)
    {
        return m_x * vector.m_x + m_y * vector.m_y + m_z * vector.m_z;
    }

    Vector3 Vector3::cross(Vector3 vector)
    {
        return Vector3(
            m_y * vector.m_z - m_z * vector.m_y,
            m_z * vector.m_x - m_x * vector.m_z,
            m_x * vector.m_y - m_y * vector.m_x
        );
    }

    /* Source: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula */
    Vector3 Vector3::rotate(Vector3 axis, double angle)
    {
        axis = axis.normalize();
        return *this * cos(angle) + axis.cross(*this) * sin(angle) + axis * axis.dot(*this) * (1 - cos(angle));
    }

    Vector3 Vector3::projection_on_plane(Vector3 normal)
    {
        return *this - projection(normal);
    }

    Vector3 Vector3::projection(Vector3 vector)
    {
        return vector * (vector.dot(*this) / vector.dot(vector));
    }

    Vector3 operator*(const Vector3& vector, double scalar)
    {
        return Vector3(
            vector.m_x * scalar,
            vector.m_y * scalar,
            vector.m_z * scalar
        );
    }

    Vector3 operator/(const Vector3& vector, double scalar)
    {
        return Vector3(
            vector.m_x / scalar,
            vector.m_y / scalar,
            vector.m_z / scalar
        );
    }

    Vector3 operator+(const Vector3& vector, double scalar)
    {
        return Vector3(
            vector.m_x + scalar,
            vector.m_y + scalar,
            vector.m_z + scalar
        );
    }

    Vector3 operator-(const Vector3& vector, double scalar)
    {
        return Vector3(
            vector.m_x - scalar,
            vector.m_y - scalar,
            vector.m_z - scalar
        );
    }

    Vector3 operator+(const Vector3& left, const Vector3& right)
    {
        return Vector3(
            left.m_x + right.m_x,
            left.m_y + right.m_y,
            left.m_z + right.m_z
        );
    }

    Vector3 operator-(const Vector3& left, const Vector3& right)
    {
        return Vector3(
            left.m_x - right.m_x,
            left.m_y - right.m_y,
            left.m_z - right.m_z
        );
    }


    std::ostream& operator<<(std::ostream& out, const Vector3& v)
    {
        out << "[" << v.m_x << ", " << v.m_y << ", " << v.m_z << "]";

        return out;
    }
}



