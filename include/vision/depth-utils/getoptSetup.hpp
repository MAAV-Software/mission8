#ifndef _GETOPT_SETUP_PROJECT_1_
#define _GETOPT_SETUP_PROJECT_1_

#include <getopt.h>
#include <iostream>

static option longOptions [] = {
	// LONG        ARGUMENT USED?     (ignore) RETURN VALUE
	{"RGB",       no_argument,       nullptr, 'r'},
	{"combined",      no_argument,       nullptr, 'c'},
	{"depth",      no_argument,       nullptr, 'd'},
	{"Points",     no_argument, nullptr, 'p'},
	{"help", no_argument, nullptr, 'h'}
};

// Sets the variables passed using the command line arguments
// parsed by getopt
// will return true if successfull, throw if there is an error,
// and return false if the flag -h or --help is passed
bool setVariables(bool &combined, bool &rgb, bool &depth, bool &points, int argc, char **argv)
{
	// All are false by default
	combined = false;
	rgb = false;
	depth = false;
	points = false;
	// temp variable that holds the output mode
	try
	{
		char choice;
		while ((choice = getopt_long(argc, argv, "rcdph", longOptions,
			nullptr)) != -1)
		{
			switch (choice){
			case 'r':
				rgb = true;
				break;
			case 'c':
				combined = true;
				break;
			case 'd':
				depth = true;
				break;
			case 'p':
				points = true;
				break;
			case 'h':
				std::cout << "Use -c for combined images, use -r for rgb images,\n";
				std::cout << "use -d for depth images, or use -p for point clouds";
				std::cout << std::endl;
				return false;
				break;
			default:
				throw "Unrecognized option\n";
				break;
			} // switch (choice)
		}
	}
	catch (const char* errorStr)
	{
		throw;
	}
	return true;
}


#endif
