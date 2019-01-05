#include <iostream>

#include <yaml-cpp/yaml.h>

#include <mainwindow.h>
#include <QApplication>

#include <common/utils/GetOpt.hpp>

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
    QApplication app(argc, argv);
    MainWindow main_window(config);
    main_window.show();

    return app.exec();
}
