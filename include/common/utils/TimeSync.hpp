#ifndef TIMESYNC_HPP
#define TIMESYNC_HPP

#include <limits>

class TimeSync
{
   public:
	TimeSync(double in_alpha1, double in_alpha2);

	long f(long diff);

	long reclock(long tivaTimeStamp, long recvTimeStamp);

	long bootstrap(long tivaTimeStamp, long recvTimeStamp);

   private:
	bool doBootstrap = true;
	long tivaTimeStamp;
	long recvTimeStamp;
	double alpha1;
	double alpha2;
	long lastGoodP;
	long lastGoodQ;
};

#endif  // TIMESYNC_HPP
