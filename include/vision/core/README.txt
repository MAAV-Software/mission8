
This document will explain how to use the camera utility classes.

The CameraInterfaceBase class is an abstract class that provides a workable interface for classes that control
working with the data recieved from the camera.

It's public methods are used as follows:

virtual void getRGB(cv::Mat& mat) const = 0;
	- gets the current RGB frame in cv::Mat form

virtual void getDepth(cv::Mat& mat) const = 0;
	- gets the current Depth frame in cv::Mat form

virtual void loadNext() = 0;
	- tells the camera to load in the data from the next (new) frame

virtual CameraInterfaceBase & operator++();
	- same as loadNext

virtual int getTag() const;
	- gets the tag associated with the given camera object (for separating out data collection)

The LegacyCameraInterface class and D400CameraInterface class both provide implementations of this interface.
The LegacyCameraInterface class works with the older realsense cameras and probably won't be very necessary moving forward.
The D400CameraInterface class works with the new realsense2 cameras.

The data-log class is used to collect test data and store it neatly in directories of binaries.
It is run as follows:

./data-log <# rs1 cams> <# rs2 cams> <directory (optional)>

A directory name can be optionally provided - if a directory that already exists is given, then everything in the
existing directory will be deleted and replaced with the data.  If the executable is run without a directory name,
then a directory with the name "ImageData" will be created and used - if one already exists (e.g. from a previous
run), then the existing directory will be made into a tarball before the data collection is run.

So, if you want to collect data using 0 old cameras and 2 new cameras and store the data in a directory called
'Test1', you would run ./data-log 0 2 Test1


The DataLogReader class allows for easy reading back in from the binaries.
To make a DataLogReader instance, simply provide it with the directory that holds all of the data, and the camera
number you want it to read from.  Then it will read back in images one by one, controlled by the increment() function.

The public methods are used as follows:

DataLogReader(std::string dir, int camNum);
	- creates DataLogReader that reads from the given directory and camera

bool getRGB(cv::Mat & mat);
	- gets the RGB frame at the current counter - returns whether it is valid

bool getDepth(cv::Mat & mat);
	- gets the Depth frame at the current counter - returns whether it is valid

void increment();
	- increments the counter (to the next frame)

void setCounter(int count);
	- sets the counter (allows for jumping around frame-wise)

