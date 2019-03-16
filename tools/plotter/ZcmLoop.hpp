#pragma once

#include "DataDict.hpp"

#include <float.h>
#include <atomic>
#include <memory>

#include <zcm/zcm-cpp.hpp>

class ZcmLoop
{
public:
    ZcmLoop(std::shared_ptr<DataDict> dict);

    /**
     * @brief Kills the run loop
     */
    ~ZcmLoop();

    /**
     * @brief Loops over all dictionary values, handles
     * receiving zcm messages and converting them to proper
     * form to add to abstract data.
     */
    void run();

    /**
     * @brief Takes utime from messages, adjusts it to elapsed
     * time and converts to seconds
     */
    double elapsedTime(const double time);

    void resetTimes();

private:
    std::atomic_bool RUNNING;
    zcm::ZCM zcm;

    std::shared_ptr<DataDict> dict_;

    double min_time = DBL_MAX;
    double max_time = -DBL_MAX;
};