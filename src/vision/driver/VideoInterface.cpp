#include "vision/driver/VideoInterface.hpp"
#include <mutex>
#include <thread>
#include "common/messages/cam_capture_t.hpp"  // TODO: Remove all uses of cam_capture_t
#include "common/messages/framebuffer_request_t.hpp"
#include "vision/depth-utils/CameraInput.hpp"
#include "vision/depth-utils/Point3f.hpp"
#include "vision/depth-utils/RGBDGetter.hpp"

using namespace pf;
typedef Point3f Point;

struct VideoInterface::VisionSimZCMData
{
	zcm::ZCM *zcmImage;
	HandlerFrame *handler;
	std::mutex *mtx;
	// set_pause_t *pause;
	// set_pause_t *cont;
	framebuffer_request_t *request;
};

class VideoInterface::HandlerFrame
{
   public:
	cam_capture_t frameData;
	void handleMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan,
					   const cam_capture_t *msg)
	{
		frameData = *msg;
	}
};

class VideoInterface::Impl
{
   public:
	virtual bool open(int index) = 0;
	virtual bool open(std::string filename) = 0;
	virtual bool isOpened() = 0;
	virtual void release() = 0;
	virtual bool grab() = 0;
	virtual bool retrieve(cv::Mat &image, int channel) = 0;
	virtual bool retrieve(pcl::PointCloud<pcl::PointXYZ> &cloud, int channel) = 0;
	virtual bool retrieve(std::vector<Point> &cloud, int channel) = 0;
	virtual bool read(cv::Mat &image) = 0;
	virtual bool read(pcl::PointCloud<pcl::PointXYZ> &cloud) = 0;
	virtual bool read(std::vector<Point> &cloud) = 0;
	virtual double get(int propID) = 0;
	virtual bool set(int propID, double value) = 0;
	virtual void init(VisionSimZCMData *&zcmDataOut) = 0;
	virtual void setContext(rs::context *_context) {}
	virtual rs::context *getContext() { return nullptr; }
};

class VideoInterface::cvVideoCapture : public VideoInterface::Impl
{
   public:
	virtual bool open(int index) { return cap.open(index); }
	virtual bool open(std::string file) { return cap.open(file); }
	virtual bool isOpened() { return cap.isOpened(); }
	virtual void release() { cap.release(); }
	virtual bool grab() { return cap.grab(); }
	virtual bool retrieve(cv::Mat &image, int channel) { return cap.retrieve(image, channel); }
	virtual bool retrieve(pcl::PointCloud<pcl::PointXYZ> &cloud, int channel) { return false; }
	virtual bool retrieve(std::vector<Point> &cloud, int channel) { return false; }
	virtual bool read(cv::Mat &image) { return cap.read(image); }
	virtual bool read(pcl::PointCloud<pcl::PointXYZ> &cloud) { return false; }
	virtual bool read(std::vector<Point> &cloud) { return false; }
	virtual double get(int propID) { return cap.get(propID); }
	virtual bool set(int propID, double value) { return cap.set(propID, value); }
	virtual void init(VisionSimZCMData *&zcmDataOut) { zcmDataOut = nullptr; }
   private:
	cv::VideoCapture cap;
};

class VideoInterface::RGBDGetterInterface : public VideoInterface::Impl
{
   public:
	virtual bool open(int index)
	{
		rgbd_get = RGBDGetter(index);
		is_open = true;
		return true;
	}
	virtual bool open(std::string file) { return false; }
	virtual bool isOpened() { return is_open; }
	virtual void release() { std::cerr << "this does nothing. \n"; }
	virtual bool grab()
	{
		++rgbd_get;
		return is_open;
	}
	virtual bool retrieve(cv::Mat &image, int channel) { return read(image); }
	virtual bool retrieve(pcl::PointCloud<pcl::PointXYZ> &cloud, int channel)
	{
		return read(cloud);
	}
	virtual bool retrieve(std::vector<Point> &cloud, int channel) { return read(cloud); }
	virtual bool read(cv::Mat &image)
	{
		rgbd_get.getRGB(image);
		return true;
	}
	virtual bool read(pcl::PointCloud<pcl::PointXYZ> &cloud)
	{
		rgbd_get.getCloud(cloud);
		return true;
	}
	virtual bool read(std::vector<Point> &cloud)
	{
		rgbd_get.getCloud(cloud);
		return true;
	}
	virtual double get(int propID)
	{
		std::cerr << "This does nothing. \n";
		return 0;
	}
	virtual bool set(int propID, double value)
	{
		std::cerr << "This does nothing. \n";
		return false;
	}
	virtual void init(VisionSimZCMData *&zcmDataOut) { zcmDataOut = nullptr; }
   private:
	RGBDGetter rgbd_get = RGBDGetter(0);
	bool is_open = false;
};

class VideoInterface::CameraInputInterface : public VideoInterface::Impl
{
   public:
	virtual bool open(int index)
	{
		if (context)
		{
			std::cout << "if context true \n";
			camera = new CameraInput(index, context);
		}
		else
		{
			std::cout << "if context false \n";
			camera = new CameraInput(index);
		}
		is_open = true;
		return true;
	}
	virtual bool open(std::string file) { return false; }
	virtual bool isOpened() { return is_open; }
	// TODO: have this release Camera in CameraInput instance??
	virtual void release() { std::cerr << "This does nothing. \n"; }
	virtual bool grab()
	{
		++(*camera);
		return true;
	}
	virtual bool retrieve(cv::Mat &image, int channel) { return read(image); }
	virtual bool retrieve(pcl::PointCloud<pcl::PointXYZ> &cloud, int channel)
	{
		return read(cloud);
	}
	virtual bool retrieve(std::vector<Point> &cloud, int channel) { return read(cloud); }
	virtual bool read(cv::Mat &image)
	{
		camera->getRGB(image);
		return true;
	}
	virtual bool read(pcl::PointCloud<pcl::PointXYZ> &cloud)
	{
		camera->getCloud(cloud);
		return true;
	}
	virtual bool read(std::vector<Point> &cloud)
	{
		camera->getCloud(cloud);
		return true;
	}
	virtual double get(int propID)
	{
		std::cerr << "This does nothing. \n";
		return 0;
	}
	virtual bool set(int propID, double value)
	{
		std::cerr << "This does nothing. \n";
		return false;
	}

	virtual void init(VisionSimZCMData *&zcmDataOut) { zcmDataOut = nullptr; }
	virtual void setContext(rs::context *_context) { context = _context; }
	virtual rs::context *getContext() { return camera->getContext(); }
	~CameraInputInterface() { delete camera; }
   private:
	rs::context *context;
	CameraInput *camera;
	bool is_open = false;
};

class VideoInterface::PhysSimInterface : public VideoInterface::Impl
{
   public:
	PhysSimInterface(VisionSimZCMData *zcmDataIn) : opened{false}, isOwner{false}
	{
		zcmData = *zcmDataIn;
		std::cerr << "before test 73\n";
		std::lock_guard<std::mutex> l(*zcmData.mtx);
		std::cerr << "after test 75\n";
	}
	PhysSimInterface() : opened{false}, isOwner{false} {}
	virtual bool open(int index)
	{
		camID = index;
		return opened = true;
	}
	virtual bool open(std::string file) { return false; }
	virtual bool isOpened() { return opened; }
	virtual void release() { std::cerr << "this does nothing. \n"; }
	virtual bool grab() { return opened; }
	virtual bool retrieve(cv::Mat &image, int channel) { return read(image); }
	virtual bool retrieve(pcl::PointCloud<pcl::PointXYZ> &cloud, int channel) { return false; }
	virtual bool retrieve(std::vector<Point> &cloud, int channel) { return false; }
	virtual bool read(cv::Mat &image)
	{
		std::lock_guard<std::mutex> l(*zcmData.mtx);
		zcmData.request->buffer_id = camID;
		zcmData.zcmImage->publish("BUFREQ", zcmData.request);
		// zcmData.zcmImage->publish("PAUSE",zcmData.pause);
		zcmData.zcmImage->handle();
		populateImage(image, zcmData.handler->frameData);
		// zcmData.zcmImage->publish("PAUSE",zcmData.cont);
		return true;
	}
	virtual bool read(pcl::PointCloud<pcl::PointXYZ> &cloud) { return false; }
	virtual bool read(std::vector<Point> &cloud) { return false; }
	virtual double get(int propID)
	{
		std::cerr << "This does nothing. \n";
		return 0;
	}
	virtual bool set(int propID, double value)
	{
		std::cerr << "This does nothing. \n";
		return false;
	}
	virtual void init(VisionSimZCMData *&zcmDataOut)
	{
		isOwner = true;
		zcmData.handler = new HandlerFrame();
		zcmData.mtx = new std::mutex();
		zcmData.zcmImage = new zcm::ZCM("ipc");
		zcmData.zcmImage->subscribe("CAM", &HandlerFrame::handleMessage, zcmData.handler);
		// zcmData.pause = new set_pause_t();
		// zcmData.pause->should_pause = true;
		// zcmData.pause->requester = "vision_driver";
		// zcmData.cont = new set_pause_t();
		// zcmData.cont->should_pause = false;
		// zcmData.cont->requester = "vision_driver";
		// Priming the zcm channels
		// zcmData.zcmImage->publish("PAUSE",zcmData.cont);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		zcmData.request = new framebuffer_request_t();
		zcmData.request->buffer_id = 0;
		// Priming the zcm
		zcmData.zcmImage->publish("BUFREQ", zcmData.request);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		zcmDataOut = &zcmData;
	}
	void populateImage(cv::Mat &image, cam_capture_t data)
	{
		cv::Mat temp = cv::Mat(data.height, data.width, CV_8UC3);
		// memcpy framedump into cv::Mat
		memcpy(temp.data, data.raw_image.data(), data.size);

		// Vertically flip the image, since OpenGL
		// and OpenCV store rows in opposite order
		image = cv::Mat(data.height, data.width, CV_8UC3);
		cv::flip(temp, image, 0);
		// cv::imwrite("output.jpg", image);
	}
	~PhysSimInterface()
	{
		if (isOwner)
		{
			delete zcmData.zcmImage;
			delete zcmData.handler;
			delete zcmData.mtx;
			// delete zcmData.pause;
			// delete zcmData.cont;
			delete zcmData.request;
		}
	}
	bool opened;
	bool isOwner;
	VisionSimZCMData zcmData;
	int camID;

   private:
};

using namespace std;
using namespace cv;

VideoInterface::VideoInterface(int type)
{
	if (!type || type == 1)
	{
		pimpl = std::make_unique<cvVideoCapture>();
	}
	else if (type == 2)
	{
		pimpl = std::make_unique<PhysSimInterface>();
	}
	else if (type == 3)
	{
		pimpl = std::make_unique<RGBDGetterInterface>();
	}
	else if (type == 4)
	{
		pimpl = std::make_unique<CameraInputInterface>();
	}
	else
	{
		std::cerr << "please input correct type \n";
		try
		{
			throw std::string("");
		}
		catch (std::string str)
		{
			throw str;
		}
	}
}

VideoInterface::VideoInterface(VisionSimZCMData *zcmDataIn)
{
	pimpl = std::make_unique<PhysSimInterface>(zcmDataIn);
}

void VideoInterface::init(VisionSimZCMData *&zcmDataOut) { return pimpl->init(zcmDataOut); }
bool VideoInterface::open(int index) { return pimpl->open(index); }
bool VideoInterface::open(std::string file) { return pimpl->open(file); }
bool VideoInterface::isOpened() { return pimpl->isOpened(); }
void VideoInterface::release() { pimpl->release(); }
bool VideoInterface::grab() { return pimpl->grab(); }
bool VideoInterface::retrieve(cv::Mat &image, int channel)
{
	return pimpl->retrieve(image, channel);
}

bool VideoInterface::read(cv::Mat &image) { return pimpl->read(image); }
bool VideoInterface::retrieve(pcl::PointCloud<pcl::PointXYZ> &cloud, int channel)
{
	return pimpl->retrieve(cloud, channel);
}

bool VideoInterface::read(pcl::PointCloud<pcl::PointXYZ> &cloud) { return pimpl->read(cloud); }
bool VideoInterface::retrieve(std::vector<pf::Point3f> &cloud, int channel)
{
	return pimpl->retrieve(cloud, channel);
}

bool VideoInterface::read(std::vector<pf::Point3f> &cloud) { return pimpl->read(cloud); }
double VideoInterface::get(int propID) { return pimpl->get(propID); }
bool VideoInterface::set(int propID, double value) { return pimpl->set(propID, value); }
void VideoInterface::spawnT2Caps(std::vector<VideoInterface *> &cap)
{
	try
	{
		if (cap.size() != 5)
		{
			std::cerr << "spawnT2Caps error, vec size wrong \n";
			throw std::string("");
		}
	}
	catch (std::string str)
	{
		throw str;
	}
	cap[0] = new VideoInterface(2);
	VisionSimZCMData *zcmData;
	cap[0]->init(zcmData);
	std::cerr << "Before test 288\n";
	{
		std::lock_guard<std::mutex> l(zcmData[0].mtx[0]);
	}
	std::cerr << "After test 291\n";
	for (int i{1}; i < 5; i++)
	{
		cap[i] = new VideoInterface(zcmData);
	}
}

rs::context *VideoInterface::getContext() { return pimpl->getContext(); }
void VideoInterface::setContext(rs::context *context) { pimpl->setContext(context); }
VideoInterface::~VideoInterface() {}
