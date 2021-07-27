#pragma once

#include "connection.hpp"

namespace KSP
{
    class Timer
    {
    private:
        krpc::Stream<double> ut_stream;
        double m_start_time;
    public:
        Timer(Connection connection);
        Timer(Connection connection, double start_time);
    public:
        double current_time;
        double last_time;
    public:
        void start();
        void set_current_time(double time);
        void set_current_time_to_ut();
        void reset();
    };

    Timer::Timer(Connection connection) : ut_stream(connection.space_center.ut_stream()), m_start_time(connection.space_center.ut())
    {

    }

    Timer::Timer(Connection connection, double start_time) : ut_stream(connection.space_center.ut_stream()), m_start_time(start_time)
    {

    }

    void Timer::start()
    {
        this->reset();
    }

    void Timer::set_current_time(double time)
    {
        last_time = current_time;
        current_time = time;
    }

    void Timer::set_current_time_to_ut()
    {
        last_time = current_time;
        current_time = ut_stream();
    }

    void Timer::reset()
    {
        m_start_time = ut_stream();
        last_time = 0.0;
        current_time = 0.0;
    }
}
