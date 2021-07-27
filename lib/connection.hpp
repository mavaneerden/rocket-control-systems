#pragma once

#include <fstream>

#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include "enums/types.hpp"

#define CONNECT_NAME "Laptop"

std::string get_address() {
	std::string address;
	std::ifstream file;

	file.open("ip-address.txt");
	file >> address;
	file.close();

	return address;
}

namespace KSP
{
    class Connection
    {
    public:
        Connection();
    public:
        krpc::Client client = krpc::connect(CONNECT_NAME, get_address());
        krpc::services::KRPC krpc = krpc::services::KRPC(&this->client);
        krpc::services::SpaceCenter space_center = krpc::services::SpaceCenter(&this->client);
    public:
        Body get_body(std::string body);
    };

    Connection::Connection()
    {
        std::cout << "Connected to server." << std::endl;
    }

    Body Connection::get_body(std::string body)
    {
        return space_center.bodies()[body];
    }
}
