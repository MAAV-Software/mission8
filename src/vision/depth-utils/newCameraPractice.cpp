#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <librealsense2/rs.hpp>
#include <iostream>
#include <fstream>
#include <cassert>
#include <cstdlib>
#include <vector>
#include <string>
#include <exception>

int main(int argc, char ** argv)
{
	assert(argc == 2);
	int numCams = atoi(argv[1]);

	rs2::context ctx;
	auto list = ctx.query_devices();
	std::cout << list.size() << " devices connected.\n";

	std::vector<rs2::pipeline> pipe(numCams);
	std::vector<rs2::config> cfg(numCams);

	const rs2::device & dev1 = list[0];
	const rs2::device & dev2 = list[1];

	rs2::pipeline p1;
	rs2::pipeline p2;

	rs2::config cfg1;
	rs2::config cfg2;

	for (int i = 0; i < numCams; ++i)
	{
		cfg[i].enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
		cfg[i].enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
		std::cout << list[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
		cfg[i].enable_device(list[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		pipe[i].start(cfg[i]);
	}

	std::vector<rs2::frameset> frames(numCams);

	// Camera warmup - dropping several first frames to let auto-exposure stabilize
    	for(int i = 0; i < 30; i++)
    	{
        	//Wait for all configured streams to produce a frame
		for (int j = 0; j < numCams; ++j)
			frames[j] = pipe[j].wait_for_frames();
    	}

	std::vector<rs2::frame> rgbFrame(numCams);
	std::vector<rs2::frame> depthFrame(numCams);
	std::ofstream file("data.txt");
	if (!file)
		throw std::runtime_error("???");
	while (true)
	{
		for (int i = 0; i < numCams; ++i)
		{
			rgbFrame[i] = frames[i].first(RS2_STREAM_COLOR);
			depthFrame[i] = frames[i].first(RS2_STREAM_DEPTH);

			if (rgbFrame[i])
			{
				auto img =(const uint16_t*)rgbFrame[i].get_data();
				cv::Mat mat = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)img, cv::Mat::AUTO_STEP);
				std::string s("RGB ");
				s += std::to_string(i);
				cv::imshow(s, mat);
			}
			if (depthFrame[i])
			{
				auto img =(const uint16_t*)depthFrame[i].get_data();
				cv::Mat mat = cv::Mat(cv::Size(640, 480), CV_16SC1, (void*)img, cv::Mat::AUTO_STEP);
				file << cv::format(mat, cv::Formatter::FMT_CSV) << std::endl;
				std::string s("Depth ");
				s += std::to_string(i);
				cv::imshow(s, mat);
			}
		}

		cv::waitKey(0);
		for (int j = 0; j < numCams; ++j)
			frames[j] = pipe[j].wait_for_frames();
	}
	

	return 0;
}
