#pragma once

#include <iostream>
#include "sleep.hpp"

namespace KSP
{
    void countdown_seconds(int seconds, std::string start, std::string end) {
        std::cout << start << std::endl;

        for (int i = seconds; i > 0; i--)
        {
            std::cout << i << std::endl;
            KSP::sleep_seconds(1);
        }

        std::cout << end << std::endl;
    }
}