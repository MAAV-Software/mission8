#include <iostream>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "VideoInterface.hpp"
#include "ImageFeed.hpp"
#include "zcm/zcm-cpp.hpp"
#include "../../../atomcore/src/zcmtypes/MsgChannels.hpp"
#include "gcsCommunication.hpp"
#include <mutex>

class CameraIdentifier
{
public:
	// Constructor for this class pass in the maximum allowable value (hsv) for
	// a camera to be considered covered
	explicit CameraIdentifier(int cameraIdThreshIn);
	// Use to identify the VideoInterfaces that are the front and bottom cameras
	// and changes their position within the vector that holds them
	// the first two arguments are the vector to be rearranged where the
	// first one is copied and not altered while the second is a reference that
	// is altered. Use green for the bottom camera and purple for
	// the front camera during startup to identify it returns the number of
	// cameras successfully identified
	int identify(std::vector<VideoInterface*> &input, std::vector<bool> &activeCams,
		std::vector<ImageFeed*> &imgFeeds, bool useGCS, zcm::ZCM* zcmPtr,
		camIdentifyHandler *handler);
private:
	// Checks which camera index is pulling black images, returns the index that is
	// if none are pulling black images returns -1
	int classifyImage(std::vector<cv::Mat> &images, std::vector<bool> &activeCams);
	// Helper function that swaps the position of two VideoInterfaces
	// in a vector of size 5
	void swapPosition(int srcIdx, int dstIdx, std::vector<VideoInterface*> &input,
		std::vector<bool> &activeCams, std::vector<ImageFeed*> &imgFeeds);
	// Holds the maximum value allowed (hsv) for a frame to be considered covered
	int cameraIdThresh;
};
