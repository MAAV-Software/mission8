#include <yaml-cpp/yaml.h>
#include <iostream>
#include <zcm/zcm-cpp.hpp>

#include <groundtruth_inertial_t.hpp>

#include "Subscriber.hpp"

using namespace std;
using zcm::ZCM;
using State = groundtruth_inertial_t;

int main(int argc, char** argv)
{
    // Determine config path
    string ypath = "../config/qualisys-config.yaml";
    if (argc > 1)
    {
        ypath = argv[1];
    }

    YAML::Node yconf;
    try
    {
        yconf = YAML::LoadFile(ypath);
    }
    catch (...)
    {
        cerr << "Could not find config file at " << ypath << endl;
        return -1;
    }

    // Exctract config info
    string sub_ip = yconf["ZMQ"]["sub_ip"].as<string>();
    int sub_port = yconf["ZMQ"]["port"].as<int>();
    string channel = yconf["ZCM"]["channel"].as<string>();
    string url = yconf["ZCM"]["url"].as<string>();

    // Construct transports
    qualisys::Subscriber<State> sub{sub_ip, sub_port};
    ZCM zcm{url};
    if (!zcm.good())
    {
        throw runtime_error{"Failed initializing ZCM"};
    }

    // Forward messages
    for (;;) {
        State state = sub.recv();
        zcm.publish(channel, &state);
    }
}