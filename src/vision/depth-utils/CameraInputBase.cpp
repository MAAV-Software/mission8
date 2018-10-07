#include "vision/depth-utils/CameraInputBase.hpp"

int CameraInputBase::GLOBAL_TAG = 0;

CameraInputBase::CameraInputBase()
{
	tag = GLOBAL_TAG;
	++GLOBAL_TAG;
}

CameraInputBase& CameraInputBase::operator++()
{
	loadNext();
	return *this;
}

int CameraInputBase::getTag() const { return tag; }
