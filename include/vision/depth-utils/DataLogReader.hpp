
#ifndef DATA_LOG_READER_H
#define DATA_LOG_READER_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class DataLogReader
{
   public:
	DataLogReader() = delete;

	// Creates a DataLogReader to read from the given directory
	DataLogReader(std::string dir, int camNum);

	// gets the RGB frame at the current count
	// returns whether a valid image was returned
	bool getRGB(cv::Mat& mat);

	// gets the Depth frame at the current count
	// returns whether a valid image was returned
	bool getDepth(cv::Mat& mat);

	// increments the counter
	void increment();

	// allows for jumping to a certain counter position
	void setCounter(int count);

   private:
	std::string directory;
	int counter;
	int cameraNumber;
};

#endif
