
#include "vision/depth-utils/DataLogReader.hpp"

#include <fstream>
#include <iostream>

using std::string;


DataLogReader::DataLogReader(std::string dir, int camNum) : directory(dir), counter(0), cameraNumber(camNum)
{

}

bool DataLogReader::getRGB(cv::Mat & mat)
{
	string filePath = directory;
	filePath += "/";
	filePath += "RGB";
	filePath += std::to_string(cameraNumber);
	filePath += "/";
	filePath += std::to_string(counter);
	filePath += ".bin";

	std::ifstream fin(filePath, std::ios::in | std::ios::binary);
	if (!fin)
		return false;
	unsigned char * data = new unsigned char[480 * 640 * 3];
	fin.read((char *)data, sizeof(unsigned char) * 480 * 640 * 3);

	std::cout << filePath << "\n";

	mat = cv::Mat(480, 640, CV_8UC3, (void *)data).clone();
	return true;
}

bool DataLogReader::getDepth(cv::Mat & mat)
{
	string filePath = directory;
	filePath += "/";
	filePath += "Depth";
	filePath += std::to_string(cameraNumber);
	filePath += "/";
	filePath += std::to_string(counter);
	filePath += ".bin";

	std::ifstream fin(filePath, std::ios::in | std::ios::binary);
	if (!fin)
		return false;
	unsigned char * data = new unsigned char[480 * 640 * 2];
	fin.read((char *)data, sizeof(unsigned char) * 480 * 640 * 2);

	mat = cv::Mat(480, 640, CV_16SC1, (void *)data).clone();
	return true;
}

void DataLogReader::increment()
{
	++counter;
}

void DataLogReader::setCounter(int count)
{
	counter = count;
}

