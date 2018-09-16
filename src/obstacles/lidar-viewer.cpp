#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "common/messages/laser_t.hpp"
#include <Eigen/Core>

#include "obstacles/ObstacleDetector.hpp"


#include <iostream>
#include <thread>
#include <vector>
#include <complex>
#include <mutex>

using namespace std;

class LidarViewer
{

public:
	LidarViewer(bool hold_in): hold(hold_in), width{1000}, height{1000}, myMat(width, height, CV_8UC3,cv::Scalar(255,255,255)), myThread{[this](){

		//std::unique_lock<std::mutex> myLock{myMutex};
		while( true )
		{
			if(hold)
			{
				std::lock_guard<std::mutex> L{myMutex};
				if(!sharedMat.empty())
				{
					imshow( "Title", sharedMat );
					cv::waitKey( 0 );
				}
			}
			else
			{
				std::lock_guard<std::mutex> L{myMutex};
				if(!sharedMat.empty())
				{
					imshow( "Title", sharedMat);
					cv::waitKey( 1 );
				}
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(10));

		}
	}}

	{

	}

	void view(const laser_t theLaser, vector<maav::ObstacleDetector::obstacle_data> ob_data)
	{

		float rad0 = theLaser.rad0;
		float radstep = theLaser.radstep;

		myMat = cv::Mat(width, height, CV_8UC3,cv::Scalar(255,255,255));

		int max_lidar_range = 5;
		for(int i = 0; i <= max_lidar_range; i++)
		{
			cv::circle(myMat, cv::Point(width/2, height/2), i * width/max_lidar_range/2, cv::Scalar(0, 0, 0), 1);
		}

		for(size_t i = 0; i < theLaser.ranges.size(); i++)
		{
			float thisRad = rad0 + i * radstep;


			int x = sin(thisRad) * width/2
				* theLaser.ranges[i] / max_lidar_range;
			int y = cos(thisRad) * width/2
				* theLaser.ranges[i] / max_lidar_range;
			//cout << "x = " << x << "\ty = "<< y << endl;

			cv::circle(myMat, cv::Point(width/2 + x, height/2 + y), 1, cv::Scalar(255, 0, 0), 5);
		}
		for(size_t i = 0; i < ob_data.size(); i++)
		{
			cv::circle(myMat, cv::Point(width/2 + ob_data[i].midpoint.x() * width/max_lidar_range/2, height/2 + ob_data[i].midpoint.y() * height/max_lidar_range/2), 30, cv::Scalar(0, 0, 255), 2);
		}

		std::unique_lock<std::mutex> myLock{myMutex};
		sharedMat = myMat.clone();
		myLock.unlock();

//		if(!myMat.empty())
//		{
//			imshow( "Title", myMat );
//			cv::waitKey( 0 );
//		}

	}

private:
	bool hold;
	int width;
	int height;
	cv::Mat myMat;
	cv::Mat sharedMat;

	std::mutex myMutex;

	std::thread myThread;


};
