#include "vision/core/CameraInterfaceBase.hpp"

using maav::vision::CameraInterfaceBase;

int CameraInterfaceBase::global_tag_ = 0;

CameraInterfaceBase::CameraInterfaceBase() : tag_{global_tag_++} {}
CameraInterfaceBase& CameraInterfaceBase::operator++()
{
    loadNext();
    return *this;
}

int CameraInterfaceBase::getTag() const { return tag_; }
