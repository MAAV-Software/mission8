#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "vision/depth-utils/DataLogReader.hpp"

using std::vector;

void open(bool rgb, std::string path)
{
	std::ifstream infile(path, std::ios::in | std::ios::binary);
	unsigned char *data;
	cv::Mat mat;
	if (rgb)
	{
		data = new unsigned char[480 * 640 * 3];
		infile.read((char *)data, sizeof(unsigned char) * 480 * 640 * 3);
		mat = cv::Mat(480, 640, CV_8UC3, (void *)data);
	}
	else
	{
		data = new unsigned char[480 * 640 * 2];
		infile.read((char *)data, sizeof(unsigned char) * 480 * 640 * 2);
		mat = cv::Mat(480, 640, CV_16SC1, (void *)data);
	}
	imshow("PHOTO", mat);
	cv::waitKey(0);

	delete[] data;
}

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: data-log-check <directory> <number of cameras>"
			<< std::endl;
		return -1;
	}

	std::string path(argv[1]);
	int numCams = atoi(argv[2]);

	std::vector<DataLogReader> readers;
	readers.reserve(static_cast<size_t>(numCams));
	for (int i = 0; i < numCams; ++i) readers.emplace_back(path, i);

	bool moreImages = true;
	while (moreImages)
	{
		moreImages = false;
		for (unsigned i = 0; i < readers.size(); ++i)
		{
			cv::Mat rgb;
			cv::Mat depth;

			if (readers[i].getRGB(rgb))
			{
				moreImages = true;
				readers[i].getDepth(depth);
				std::string title("Camera ");
				title += std::to_string(i);
				imshow(title + " RGB", rgb);
				imshow(title + " Depth", depth);
				cv::waitKey(0);
				readers[i].increment();
			}
		}
	}

	return 0;
}
