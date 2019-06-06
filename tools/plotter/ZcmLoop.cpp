#include "ZcmLoop.hpp"
#include "AbstractData.hpp"
#include "DataDict.hpp"
#include "ZcmConversion.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/ZCMHandler.hpp>

using namespace std::chrono;

ZcmLoop::ZcmLoop(std::shared_ptr<DataDict> dict, YAML::Node config)
    : RUNNING{true}, zcm{config["zcm-url"].as<std::string>()}, dict_(dict)
{
    // Initialize the min and max times;
    resetTimes();
}

ZcmLoop::~ZcmLoop() { RUNNING = false; }

double ZcmLoop::elapsedTime(const double time)
{
    if (time < min_time)
    {
        min_time = time;
    }

    if (time > max_time)
    {
        max_time = time;
    }

    if (time < max_time - 5)
    {
        for (auto& ad : dict_.get()->dict)
        {
            ad.second->purgeGraphs();
        }
        resetTimes();
        return 0;
    }

    return time - min_time;
}

void ZcmLoop::resetTimes()
{
    min_time = DBL_MAX;
    max_time = -DBL_MAX;
}
