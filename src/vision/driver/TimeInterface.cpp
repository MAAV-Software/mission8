#include "vision/driver/TimeInterface.hpp"

#include "common/messages/MsgChannels.hpp"

class TimeInterface::HandlerSimTime
{
	public:
		int64_t currentTime;
		std::mutex mtx; // For getting or changing current time
		// Default constructor prevents fatal errors
		HandlerSimTime() : currentTime {0} { }
		void handleMessage(const zcm::ReceiveBuffer *rbuf,
			const std::string &chan, const sim_time_t *msg)
		{
			std::lock_guard<std::mutex> l(mtx);
			currentTime = msg->utime / 1000;
		}
};

class TimeInterface::Impl
{
	public:
		virtual void init() = 0;
		virtual void sleep(int duration) = 0;
		virtual void getTime(int64_t &time) = 0;
};

class TimeInterface::NormalTime : public TimeInterface::Impl
{
	public:
		virtual void init() { std::cerr << "Does nothing\n"; }
		virtual void sleep(int duration)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(
				duration));
		}
		virtual void getTime(int64_t &time)
		{
			time = std::chrono::duration_cast<std::chrono::
   				microseconds>(std::chrono::system_clock::now().
   				time_since_epoch()).count();
		}
};

class TimeInterface::SimTimeInterface : public TimeInterface::Impl
{
	private:
		HandlerSimTime handlerTime;
		std::map<int64_t,std::pair<std::condition_variable*,std::mutex*>
			> conditionVariables;
		std::map<int64_t,bool> cvBools;
		std::mutex releaseMutex;
		std::condition_variable releaseCv;
		std::atomic<bool> releaseBool;
	public:
		SimTimeInterface();
		static void zcmThread(HandlerSimTime *handler,
			SimTimeInterface *simTimer);
		virtual void init();
		virtual void sleep(int duration);
		virtual void getTime(int64_t &time);
		void release();
};

TimeInterface::SimTimeInterface::SimTimeInterface()
{
	releaseBool = false;
}

void TimeInterface::SimTimeInterface::zcmThread(HandlerSimTime
	*handler, SimTimeInterface *simTimer)
{
	zcm::ZCM zcm {"ipc"};
	zcm.subscribe(maav::SIM_TIME_CHANNEL,&HandlerSimTime::handleMessage,handler);
	while (true)
	{
		zcm.handle();
		simTimer->release();
	}
}

void TimeInterface::SimTimeInterface::init()
{
	std::thread(zcmThread,&handlerTime,this).detach();
}

void TimeInterface::SimTimeInterface::sleep(int duration)
{
	int64_t inTime;
	std::condition_variable* cvPtr;
	{
		std::lock_guard<std::mutex> l(handlerTime.mtx);
		inTime = handlerTime.currentTime + duration;
		conditionVariables.insert(std::pair<int64_t,std::
			pair<std::condition_variable*,std::mutex*> > (inTime,
			std::pair<std::condition_variable*,std::mutex*>(
			new std::condition_variable(), new std::mutex())));
		cvBools.insert(std::pair<int64_t,bool>(inTime, false));
		cvPtr = conditionVariables[inTime].first;
	}
	std::unique_lock<std::mutex> lck(conditionVariables[inTime].second[0]);
	bool tempPred {false};
	while (!tempPred)
	{
		cvPtr[0].wait(lck);
		std::lock_guard<std::mutex> l(handlerTime.mtx);
		tempPred = cvBools[inTime];
	}
	releaseBool = true;
	releaseCv.notify_one();
}

void TimeInterface::SimTimeInterface::getTime(int64_t &time)
{
	std::lock_guard<std::mutex> l(handlerTime.mtx);
	time = handlerTime.currentTime;
}

void TimeInterface::SimTimeInterface::release()
{
	handlerTime.mtx.lock(); // locks mtx
	int64_t thisTime;
	bool incremented {false};
	/*
	for (auto itr = conditionVariables.begin(); itr != conditionVariables.
		end() itr = conditionVariables.begin())
	*/
	auto itr = conditionVariables.begin();
	while (itr != conditionVariables.end())
	{
	if ((*itr).first <= handlerTime.currentTime)
	{
		thisTime = (*itr).first;
		cvBools[thisTime] = true;
		(*itr).second.first[0].notify_one();
		std::unique_lock<std::mutex> lck(releaseMutex);
		handlerTime.mtx.unlock(); // unlocks mtx
		while (!releaseBool)
		{
			releaseCv.wait(lck);
		}
		handlerTime.mtx.lock(); // locks mtx
		releaseBool = false;
		cvBools.erase(thisTime);
		delete (*itr).second.first;
		delete (*itr).second.second;
		++itr;
		incremented = true;
		conditionVariables.erase(thisTime);
	}
	if (!incremented)
	{
		++itr;
		incremented = false;
	}
	}
	handlerTime.mtx.unlock(); // unlocks at end of function
}

TimeInterface::TimeInterface(int type)
{
	if (!type || type == 1)
	{
		pimpl = std::make_unique<NormalTime>();
	}
	else if (type == 2)
	{
		pimpl = std::make_unique<SimTimeInterface>();
	}
	else
	{
		std::cerr << "Enter a correct type please. exitting...\n";
		try
		{
			throw 5;
		}
		catch (int num)
		{
			throw num;
		}
	}
}

void TimeInterface::init()
{
	pimpl->init();
}

void TimeInterface::sleep(int duration)
{
	pimpl->sleep(duration);
}

void TimeInterface::getTime(int64_t &time)
{
	pimpl->getTime(time);
}

void TimeInterface::createType2(TimeInterface *&timer)
{
	timer = new TimeInterface(2);
	timer->init();
}

TimeInterface::~TimeInterface() {}
