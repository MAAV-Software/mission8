#include <common/utils/GetOpt.hpp>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <memory>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rapidjson/document.h>
#include <string>
#include <streambuf>
#include <utility>
#include <vector>
#include <zcm/zcm-cpp.hpp>

#include "gnc/planner/Astar.hpp"
#include "gnc/planner/Path.hpp"
#include "gnc/Planner.hpp"
#include "gnc/measurements/Waypoint.hpp"
#include "gnc/State.hpp"

using std::string;
using std::ifstream;
using std::cout;
using std::endl;
using std::cerr;
using std::vector;
using std::pair;
using std::shared_ptr;
using std::make_shared;

using octomap::OcTree;
using octomap::point3d;

using rapidjson::Document;
using rapidjson::GenericArray;

using Eigen::Vector3d;

using cv::Mat;
using cv::rectangle;
using cv::circle;
using cv::Point;
using cv::Scalar;
using cv::arrowedLine;

using maav::gnc::Path;
using maav::gnc::Waypoint;
using maav::gnc::State;
using maav::gnc::Planner;

// The number of columns and rows in the Mat used for rendering
constexpr int MAT_WIDTH = 1920;
constexpr int MAT_HEIGHT = 1080;

shared_ptr<OcTree> createOctomap(Document& blueprint);
vector<Vector3d> extractPath(Document& doc);
vector<pair<Vector3d, Vector3d> > extractWalls(Document& doc);
void renderWall(Mat& arena, pair<Vector3d, Vector3d>& wall);


int main(int argc, char** argv) {
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('m', "map", "NO DEFAULT", "Path to the map json file to use in the render.");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        gopt.printHelp();
        return 1;
    }

    string json_name = gopt.getString("map");
    ifstream fin(json_name);
    string map_spec;

    // Read the file into the map_spec string
    fin.seekg(0, std::ios::end);
    map_spec.reserve(fin.tellg());
    fin.seekg(0, std::ios::beg);

    map_spec.assign((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());

    cout << map_spec << endl;
    
    Document doc;
    doc.Parse(map_spec.c_str());

    // create an octomap from the config specifications
    shared_ptr<OcTree> tree = createOctomap(doc);
    cout << "octree created" << endl;

    GenericArray point1 = doc["start"].GetArray();
    GenericArray point2 = doc["goal"].GetArray();
    auto initPt = Vector3d(point1[0].GetDouble(), point1[1].GetDouble(), point1[2].GetDouble());
    auto goalPt = Vector3d(point2[0].GetDouble(), point2[1].GetDouble(), point2[2].GetDouble());

    // generate a path with astar
    Planner planner(""); //TODO add config for planner
    planner.update_target(Waypoint(goalPt, Vector3d(0,0,0), 0));
    State initial = State::zero(0);
    initial.position() = initPt;
    planner.update_state(initial);
    planner.update_map(tree);
    // run A*
    Path p = planner.get_path();
    planner.print_path(p);

    vector<Vector3d> path;
    std::transform(p.waypoints.cbegin(), p.waypoints.cend(), std::back_inserter(path), 
        [](const Waypoint& w) {
        Vector3d pos = w.position;
        return pos;
    });


    // create renderings
    vector<pair<Vector3d, Vector3d>> eigen_walls = extractWalls(doc);

    // Every point is multiplied by this scaling such that the space of the
    // rendering is most effectively used while not allowing the arena to bleed outside the frame
    double scaling;
    {
        GenericArray arena_size = doc["arena-size"].GetArray();
        double arena_height = arena_size[0].GetDouble();
        double arena_width = arena_size[1].GetDouble();
        const double width_scale = MAT_WIDTH / arena_width;
        const double height_scale = MAT_HEIGHT / arena_height;
        // Take the smallest of the height and width scalings
        scaling = width_scale < height_scale ? width_scale : height_scale;
    }

    cout << "Scaling: " << scaling << endl;

    // TODO this needs to be in its own function so that it can be done to any point I pass in
    // TODO this actually can't be shared because for some reason the rectangles are opposite
    // Correct the eigen wall entries with the scaling and offset
    for (auto& wall: eigen_walls)
    {
        // Correct the scaling
        wall.first[0] *= scaling;
        wall.first[1] *= scaling;
        wall.second[0] *= scaling;
        wall.second[1] *= scaling;
        // Correct the offset
        wall.first[0] += MAT_HEIGHT / 2;
        wall.first[1] += MAT_WIDTH / 2;
        wall.second[0] += MAT_HEIGHT / 2;
        wall.second[1] += MAT_WIDTH / 2;
    }

    // The Mat which rendering is done on
    Mat arena(MAT_HEIGHT, MAT_WIDTH, CV_8UC3);
    // Fill the entire thing so that there is contrast
    rectangle(arena, cv::Point2d(0, 0), cv::Point2d(1079, 1919), cv::Scalar(0.0), CV_FILLED);

    // Render the walls onto the arena
    for (auto& wall: eigen_walls)
    {
        renderWall(arena, wall);
    }

    // // For testing purposes read in a path from the config
    // vector<Vector3d> path = extractPath(doc);

    // Adjust the path astar spit out with the scaling and offset
    for (auto& point : path)
    {
        // Correct the scaling
        point[0] *= scaling;
        point[1] *= scaling;
        // Correct the offset
        point[0] += MAT_HEIGHT / 2;
        point[1] += MAT_WIDTH / 2;
    }
    // Render the path that astar spit out
    // Render the starting point as a green dot
    {
        cv::Point first(path.front()[1], path.front()[0]);
        cv::circle(arena, first, 10, cv::Scalar(0, 250, 0), CV_FILLED);
    }
    // Render the ending point as a red dot
    {
        cv::Point last(path.back()[1], path.back()[0]);
        cv::circle(arena, last, 10, cv::Scalar(0, 0, 250), CV_FILLED);
    }
    cout << path.size() << endl;
    // Render the path as a series of connected line segments
    for (size_t i = 1; i < path.size(); ++i)
    {
        Vector3d& pt1 = path[i - 1];
        Vector3d& pt2 = path[i];
        arrowedLine(arena, Point(pt1[1], pt1[0]), Point(pt2[1], pt2[0]), Scalar(200, 100, 200), 3);
    }
    
    // Display the end result
    cv::namedWindow("A* Test Rendering", cv::WINDOW_AUTOSIZE);
    imshow("A* Test Rendering", arena);
    cv::waitKey(0);

}

// helper function for vector to point3d
point3d VecToPoint3d(const Vector3d& v)
{
    return point3d(v[0], v[1], v[2]);
}

shared_ptr<OcTree> createOctomap(Document& blueprint)
{   
    double resolution = blueprint["resolution"].GetDouble();
    GenericArray arena_size = blueprint["arena-size"].GetArray();
    auto tree = make_shared<OcTree>(OcTree(resolution));
    std::vector<double> dims = {arena_size[0].GetDouble(), arena_size[1].GetDouble(), 
                                arena_size[2].GetDouble()};
    // insert measurements of 'free' environmenet
    for (float x=-dims[0]/2; x < dims[0]/2; x+=resolution ) {
        for (float y=-dims[1]/2; y < dims[1]/2; y+=resolution) {
            for (float z=0; z > dims[2]; z-=resolution) {
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
    vector<pair<Vector3d, Vector3d>> walls = extractWalls(blueprint);
    for(auto &wall: walls)
    {
        addObject(VecToPoint3d(wall.first), VecToPoint3d(wall.second));
    }

    return tree;
}

// Used for testing, extracts the path from the map spec
vector<Vector3d> extractPath(Document& doc)
{
    GenericArray path = doc["path"].GetArray();
    vector<Vector3d> extracted;
    extracted.reserve(path.Size());
    for (auto& point : path) 
    {
        extracted.emplace_back(point[0].GetDouble(), point[1].GetDouble(), point[2].GetDouble());
    }
    return extracted;
}

// Used to extract the pairs of 3d points that represent the walls from the JSON dom
vector<pair<Vector3d, Vector3d> > extractWalls(Document& doc)
{
    GenericArray walls = doc["walls"].GetArray();
    vector<pair<Vector3d, Vector3d> > eigen_walls;
    eigen_walls.reserve(walls.Size());
    for (auto& wall : walls)
    {
        GenericArray point1 = wall[0].GetArray();
        GenericArray point2 = wall[1].GetArray();
        eigen_walls.emplace_back(
            Vector3d(point1[0].GetDouble(), point1[1].GetDouble(), point1[2].GetDouble()),
            Vector3d(point2[0].GetDouble(), point2[1].GetDouble(), point2[2].GetDouble())
        );
    }
    return eigen_walls;
}

void renderWall(Mat& arena, pair<Vector3d, Vector3d>& wall)
{
    rectangle(arena, cv::Point2d(wall.first[1], wall.first[0]), cv::Point2d(wall.second[1], 
        wall.second[0]), cv::Scalar(250), CV_FILLED);
}