#include "gcs/GCS.hpp"
#include "gcs/GCSConsts.hpp"
#include "common/utils/debug.hpp"
#include "common/utils/GetOpt.hpp"

#include <string>
#include <iostream>
#include <gtkmm/application.h>
#include <glibmm/main.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace YAML;

int main(int argc, char** argv)
{
	GetOpt gopt;
	gopt.addBool('h', "help", false, "Display help text");
	gopt.addString('c', "config", "../../config/gcs-config.yaml",
		"Location of YAML config file");

	if(!gopt.parse(argc, argv, 1) || gopt.getBool("help")) {
		cout << "Usage: " << argv[0] << " [options]" << endl;
		gopt.printHelp();
		return 1;
	}

	const Node& config = LoadFile(gopt.getString("config"));
	const Node& pos = config["Tuning"]["pos"];
	const Node& rate = config["Tuning"]["rate"];

	const maav::gcs::GCSConsts& CONSTS{config["Spacing"]["small"].as<int>(),
		config["Spacing"]["med"].as<int>(),
		config["Spacing"]["large"].as<int>(),
		config["Timeout"]["quiet"].as<int>(),
		config["Timeout"]["down"].as<int>(),
		{pos["x"][0].as<double>(), pos["x"][1].as<double>(), pos["x"][2].as<double>()},
		{pos["y"][0].as<double>(), pos["y"][1].as<double>(), pos["y"][2].as<double>()},
		{pos["z"][0].as<double>(), pos["z"][1].as<double>(), pos["z"][2].as<double>()},
		{pos["yaw"][0].as<double>(), pos["yaw"][1].as<double>(), pos["yaw"][2].as<double>()},
		{rate["x"][0].as<double>(), rate["x"][1].as<double>(), rate["x"][2].as<double>()},
		{rate["y"][0].as<double>(), rate["y"][1].as<double>(), rate["y"][2].as<double>()},
		{rate["z"][0].as<double>(), rate["z"][1].as<double>(), rate["z"][2].as<double>()},
		gopt.getString("config"), config["url"].as<string>()};

	auto app = Gtk::Application::create();
	MAAV_DEBUG("Starting MAAV Ground Control Station");
	maav::gcs::GCS win{CONSTS.URL.c_str(), CONSTS};
	return app->run(win);
}
