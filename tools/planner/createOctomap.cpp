/*
* This file creates simple octomaps for testing. Takes in yaml file and 
* outputs a .ot file. The created octomap can be published to the planner
* pipeline via publish-octomap executable.
*
* Created octomaps can be found in the datasets folder of the MAAV team GDrive
* Author: rywunder
*/

#include <algorithm>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <string>
#include <sstream>
#include <memory>

#include <yaml-cpp/yaml.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

// helper function for vector to point3d
point3d VecToPoint3d(const vector<double>& v)
{
	return point3d(v[0], v[1], v[2]);
}

shared_ptr<OcTree> createOctomap(YAML::Node blueprint)
{	
	double resolution = blueprint["resolution"].as<double>();
	auto tree = make_shared<OcTree>(OcTree(resolution));
	std::vector<double> dims = blueprint["arena-size"].as<std::vector<double>>();
	// insert measurements of 'free' environment
	for (float x=-dims[0]/2; x < dims[0]/2; x+=resolution ) {
		for (float y=-dims[1]/2; y < dims[1]/2; y+=resolution) {
			for (float z=0; z > dims[2]; z+=resolution) {
				point3d endpoint ((float) x, (float) y, (float) z);
         		tree->updateNode(endpoint, false);  
			}
		}
	}

	// adds the object to the tree using a bbx_iterator
	auto addObject = [&tree](const point3d& min, const point3d& max)
	{
		for(OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(min,max),
	       end=tree->end_leafs_bbx(); it!= end; ++it)
		{
		  tree->updateNode(it.getCoordinate(), true);
		}
	};

	// create the floor
	addObject(point3d(-dims[0]/2, -dims[1]/2, 0), 
			  point3d(-dims[0]/2, -dims[1]/2, dims[2]));

	// create the walls
	vector<vector<vector<double> > > walls;
	for(auto &wall: walls)
	{
		addObject(VecToPoint3d(wall[0]), VecToPoint3d(wall[1]));
	}

	return tree;
}

int main(int argc, char** argv)
{
    if(argc != 3)
    {
    	 cerr << "USAGE: ./tool-create-octomap <yamlFilePath> <outputFilePath>";
    }

    string fileOut = argv[2];

    YAML::Node buildNode = YAML::LoadFile(string(argv[1]));
    auto octomap = createOctomap(buildNode);
    octomap->write(fileOut);
}