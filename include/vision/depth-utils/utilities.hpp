#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "common/messages/depth_image_t.hpp"
#include "common/messages/rgbd_image_t.hpp"

namespace maav::vision
{
// Returns a cv::Mat that is a clone of the memory specified
// Where the memory is interpretted as an rgb image (bgr) (CV8U_C3)
// memPtr is the pointer to the start of the memory,
// the memory pointed to by memPtr must be layed out like
// in a cv::Mat, width and height are the width and height of
// the image that this memory represents
cv::Mat wrapInRGBMat(void* mem_ptr, int width, int height);
// Returns a cv::Mat that is a clone of the memory specified
// Where the memory is interpretted as a depth image (CV16U_C1)
// memPtr is the pointer to the start of the memory,
// the memory pointed to by memPtr must be layed out like
// in a cv::Mat, width and height are the width and height of
// the image that this memory represents
cv::Mat wrapInDepthMat(void* mem_ptr, int width, int height);

void rgbdToZcmType(const cv::Mat& rgb, const cv::Mat& depth, rgbd_image_t& zcm_img);

void zcmTypeToRgbd(const rgbd_image_t& zcm_img, cv::Mat& rgb, cv::Mat& depth);
}
