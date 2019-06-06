#include <iostream>
#include <memory>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <QApplication>

#include <common/utils/GetOpt.hpp>
#include "DataDict.hpp"
#include "FlightInstruments.hpp"
#include "FlightInstrumentsWindow.hpp"
#include "LinePlotWindow.h"
#include "ListWindow.h"
#include "ZcmLoop.hpp"

int main(int argc, char *argv[])
{
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/tools/plotter-config.yaml", "Path to YAML config");
    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(gopt.getString("config"));
    }
    catch (...)
    {
        std::cout
            << "Please provide correct path to configuration file with \"-c <path-to-config>\"\n";
        return 1;
    }

    // Create QApplication before all other objects
    QApplication app(argc, argv);

    // DataDict stores all data for plotting organized in a dictionary
    std::shared_ptr<DataDict> dict(new DataDict());

    // ZcmLoop runs in another thread and feeds zcm messages into the dictionary
    ZcmLoop loop(dict, config);

    ListWindow list_window(dict, config);
    list_window.show();

    std::cout << "Here" << std::endl;

    FlightInstrumentsWindow flight_instruments(config);
    flight_instruments.show();

    std::thread zcm_loop = std::thread(&ZcmLoop::run, &loop);
    std::thread dict_loop = std::thread(&DataDict::run, dict);

    return app.exec();
}
