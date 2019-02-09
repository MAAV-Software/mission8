
#include "vision/depth-utils/DataLogReader.hpp"

#include <fstream>
#include <iostream>

using std::string;

DataLogReader::DataLogReader(std::string dir, int camNum)
    : directory_(dir), counter_(0), camera_number_(camNum)
{
}

bool DataLogReader::getRGB(cv::Mat &mat)
{
    string filePath = directory_;
    filePath += "/";
    filePath += "RGB";
    filePath += std::to_string(camera_number_);
    filePath += "/";
    filePath += std::to_string(counter_);
    filePath += ".bin";

    std::ifstream fin(filePath, std::ios::in | std::ios::binary);
    if (!fin) return false;
    unsigned char *data = new unsigned char[480 * 640 * 3];
    fin.read((char *)data, sizeof(unsigned char) * 480 * 640 * 3);

    std::cout << filePath << "\n";

    mat = cv::Mat(480, 640, CV_8UC3, (void *)data).clone();
    return true;
}

bool DataLogReader::getDepth(cv::Mat &mat)
{
    string filePath = directory_;
    filePath += "/";
    filePath += "Depth";
    filePath += std::to_string(camera_number_);
    filePath += "/";
    filePath += std::to_string(counter_);
    filePath += ".bin";

    std::ifstream fin(filePath, std::ios::in | std::ios::binary);
    if (!fin) return false;
    unsigned char *data = new unsigned char[480 * 640 * 2];
    fin.read((char *)data, sizeof(unsigned char) * 480 * 640 * 2);

    mat = cv::Mat(480, 640, CV_16SC1, (void *)data).clone();
    return true;
}

void DataLogReader::increment() { ++counter_; }
void DataLogReader::setCounter(int count) { counter_ = count; }
