#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rapidjson/document.h>

#include <zcm/zcm-cpp.hpp>

#include <common/utils/GetOpt.hpp>

#include <Eigen/Core>

#include <fstream>
#include <iostream>
#include <string>
#include <streambuf>
#include <vector>
#include <utility>

using std::string;
using std::ifstream;
using std::cout;
using std::endl;
using std::cerr;
using std::vector;
using std::pair;

using rapidjson::Document;
using rapidjson::GenericArray;

using Eigen::Vector3d;

using cv::Mat;
using cv::rectangle;

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

// TODO Just put all of rywunder's code here

void renderWall(Mat& arena, pair<Vector3d, Vector3d>& wall)
{
    rectangle(arena, cv::Point2d(wall.first[1], wall.first[0]), cv::Point2d(wall.second[1], 
        wall.second[0]), cv::Scalar(250), CV_FILLED);
}

// The number of columns and rows in the Mat used for rendering
constexpr int MAT_WIDTH = 1920;
constexpr int MAT_HEIGHT = 1080;

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

    vector<pair<Vector3d, Vector3d>> eigen_walls = extractWalls(doc);

    cout << eigen_walls.size() << endl;

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
    {
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
    }

    // The Mat which rendering is done on
    Mat arena(MAT_HEIGHT, MAT_WIDTH, CV_8UC3);
    // Fill the entire thing so that there is contrast
    rectangle(arena, cv::Point2d(0, 0), cv::Point2d(1079, 1919), cv::Scalar(0.0), CV_FILLED);

    // Render the walls onto the arena
    for (auto& wall: eigen_walls)
    {
        renderWall(arena, wall);
        // rectangle(arena, cv::Point2d(wall.first[0], wall.first[1]), cv::Point2d(wall.second[0], 
        //    wall.second[1]), cv::Scalar(250), CV_FILLED);
        // Display the rendered walls
    }
    // Display the rendered walls
    cv::namedWindow("A* Test Rendering", cv::WINDOW_AUTOSIZE);
    imshow("A* Test Rendering", arena);
    cv::waitKey(0);

}