#pragma once

#include <math.h>
#include "timer.hpp"

namespace KSP
{
    class PID
    {
    public:
        PID(Connection connection, double kP, double kI, double kD);
    private:
        double kP;
        double kI;
        double kD;
        double last_error;
        double total_error;
        double previous_value;
        Timer timer;
    public:
        void start();
        double step(double target, double current, double I_bound = 1.0);
        void reset_error();
    };

    PID::PID(Connection connection, double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD), timer(Timer(connection))
    {

    }

    void PID::start()
    {
        timer.set_current_time_to_ut();
        reset_error();
    }

    double PID::step(double target, double current, double I_bound)
    {
        double P = target - current;
        double I = 0.0;
        double D = 0.0;
        double dt = timer.current_time - timer.last_time;
        double result;

        timer.set_current_time_to_ut();

        if (std::abs(P) < 0.002)
        {
            return previous_value;
        }

        if (timer.last_time > 0)
        {
            I = total_error + ((P + last_error) / 2 * (dt));
            D = (P - last_error) / (dt);
        }

        if (kI > 0)
        {
            I = std::min(I_bound / kI, std::max(-I_bound/ kI, I));
        }

        last_error = P;
        total_error = I;

        std::cout << "P: " << P * kP << "   I: " << I * kI << "   D: " << D * kD << std::endl;

        std::cout << "I: " << I << std::endl;

        result = P * kP + I * kI + D * kD;
        previous_value = result;

        return result;
    }

    void PID::reset_error()
    {
        total_error = 0.0;
    }
}
