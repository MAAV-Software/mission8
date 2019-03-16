#include "DataDict.hpp"

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono;

DataDict::DataDict()
{
    initializeDictionary_generated();
}

void DataDict::run()
{
    RUNNING = true;
    while (RUNNING)
    {
        for (const auto& data : dict)
        {
            data.second->plotLines();
        }
        std::this_thread::sleep_for(33ms);
    }
}