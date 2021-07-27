#pragma once

#include <chrono>
#include <thread>

namespace KSP
{
    void sleep_milliseconds(unsigned int miliseconds)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(miliseconds));
    }

    void sleep_seconds(unsigned int seconds)
    {
        sleep_milliseconds(seconds * 1000);
    }
}
