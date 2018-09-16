#ifndef _FIND_DELAY_
#define _FIND_DELAY_
#include <fstream>
#include <iostream>
#include <cstdint>
#include <chrono> 
#include <string>

class findDelay
{
public:
	findDelay() : findDelay(std::string("")) {  }
	findDelay(std::string fileOut) : firstTime {0}, lastTime {0}
	{
		std::string filename("Outdelays");
		filename += fileOut;
		filename += ".txt";
		fout.open(filename.c_str());
	}
	void first(std::string nameIn)
	{
		delayID = nameIn;
		firstTime = std::chrono::duration_cast<std::chrono::
   			milliseconds>(std::chrono::system_clock::now().
   			time_since_epoch()).count();
	}
	void last()
	{
		lastTime = std::chrono::duration_cast<std::chrono::
   			milliseconds>(std::chrono::system_clock::now().
   			time_since_epoch()).count();
		write();
	}
private:
	void write()
	{
		fout << delayID << ": ";
		fout << lastTime - firstTime << '\n';
	}
	std::string delayID;
	int64_t firstTime;
	int64_t lastTime;
	std::ofstream fout;
};

#endif
