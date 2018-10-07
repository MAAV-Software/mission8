#include "vision/driver/CameraIdentifier.hpp"

using namespace std;
using namespace cv;

CameraIdentifier::CameraIdentifier(int cameraIdThreshIn) : cameraIdThresh{cameraIdThreshIn} {}
int CameraIdentifier::identify(vector<VideoInterface *> &input, vector<bool> &activeCams,
							   vector<ImageFeed *> &imgFeeds, bool usingGCS, zcm::ZCM *zcmPtr,
							   camIdentifyHandler *handler)
{
	vector<Mat> images(5);
	int numFound{0};
	int srcIdx{0};
	int numActiveCams{0};
	int dstIdx{0};
	string tempStr;
	camera_disc_t msg;
	for (int i{0}; i < 5; ++i)
	{
		if (activeCams[i])
		{
			++numActiveCams;
		}
	}
	cout << "Use GCS to send camera idx you are trying to identify\n";
	cout << "Or if in non GCS mode, type in the index into the terminal\n";
	cout << "Make sure index is less than 6, if greater will continue cv.wait\n";
	while (numFound < numActiveCams)
	{
		if (usingGCS)
		{
			handler->numFound = 6;
			unique_lock<mutex> lck(handler->mtx);
			while (handler->numFound > 5)
			{
				handler->cv.wait(lck);
			}
			dstIdx = handler->numFound;
			handler->numFound = 0;
		}
		else
		{
			cin >> tempStr;
			dstIdx = stoi(tempStr);
		}
		cout << "Finding camera " << dstIdx << '\n';
		// Makes sure that the frame used is in fact a new one
		for (int i{0}; i < 10; ++i)
		{
			for (int ii{0}; ii < 5; ++ii)
			{
				if (!activeCams[ii])
				{
					continue;
				}
				else if (input[ii]->grab())
				{
					input[ii]->retrieve(images[ii]);
				}
			}
		}
		for (bool found{false}; !found;)
		{
			for (int i{0}; i < 5; ++i)
			{
				if (!activeCams[i])
				{
					continue;
				}
				while (!input[i]->grab())
				{
				}
				input[i]->retrieve(images[i]);
			}
			srcIdx = classifyImage(images, activeCams);
			if (srcIdx == -1)
			{
				continue;
			}
			swapPosition(srcIdx, dstIdx, input, activeCams, imgFeeds);
			++numFound;
			found = true;
			msg.numCameras = numFound;
			zcmPtr->publish(maav::CAMERA_DISC_STAT, &msg);
			cout << "Found camera " << dstIdx << '\n';
		}
	}
	return numFound;
}

void CameraIdentifier::swapPosition(int srcIdx, int dstIdx, vector<VideoInterface *> &input,
									vector<bool> &activeCams, vector<ImageFeed *> &imgFeeds)
{
	// Check if srcIdx is same as new, return if same
	if (srcIdx == dstIdx)
	{
		return;
	}
	// Declare temp pointers
	VideoInterface *tempPtr;
	ImageFeed *tempImgFeedPtr;
	// swap VideoInterface
	tempPtr = input[dstIdx];
	input[dstIdx] = input[srcIdx];
	input[srcIdx] = tempPtr;
	// swap activeCams bool
	bool tempBool;
	tempBool = activeCams[dstIdx];
	activeCams[dstIdx] = activeCams[srcIdx];
	activeCams[srcIdx] = tempBool;
	// Swap ImageFeeds
	tempImgFeedPtr = imgFeeds[dstIdx];
	imgFeeds[dstIdx] = imgFeeds[srcIdx];
	imgFeeds[srcIdx] = tempImgFeedPtr;
}

// Pass a vector of size 5 of cv mats and a vector of 5 bools
int CameraIdentifier::classifyImage(vector<Mat> &images, vector<bool> &activeCams)
{
	vector<Mat> hsvImages(5);
	vector<Mat> binary(5);
	for (int i{0}; i < 5; ++i)
	{
		if (!activeCams[i])
		{
			continue;
		}
		cvtColor(images[i], hsvImages[i], COLOR_BGR2HSV);
		inRange(hsvImages[i], Scalar(0, 0, 0), Scalar(255, 255, cameraIdThresh), binary[i]);
		if ((sum(binary[i]))[0] > 230400)
		{
			return i;
		}
	}
	return -1;
}
