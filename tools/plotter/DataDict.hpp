#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "AbstractData.hpp"

class DataDict
{
public:
    DataDict();

    /**
     * @brief Kills the run loop
     */
    ~DataDict()
    {
        RUNNING = false;
    }

    void initializeDictionary_generated();

    /**
     * @brief Loops over all abstact data types in the
     * dict and runs plot lines
     */
    void run();

    std::map<std::string, std::shared_ptr<AbstractData>> dict;

    std::atomic<bool> RUNNING;
};