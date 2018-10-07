#include "common/utils/TimeSync.hpp"

TimeSync::TimeSync(double alpha1_, double alpha2_) : alpha1(alpha1_), alpha2(alpha2_) {}
long TimeSync::f(long diff)
{
	long case1 = (alpha2 * diff) / (1 + alpha2);
	long case2 = (alpha1 * diff) / (1 - alpha1);
	if (case1 > case2)
	{
		return case1;
	}
	return case2;
}

long TimeSync::reclock(long tivaTimeStamp, long recvTimeStamp)
{
	if (doBootstrap)
	{
		return bootstrap(tivaTimeStamp, recvTimeStamp);
	}
	else
	{
		if (tivaTimeStamp - recvTimeStamp - f(0) >=
			lastGoodP - lastGoodQ - f(tivaTimeStamp - lastGoodP))
		{
			lastGoodP = tivaTimeStamp;
			lastGoodQ = recvTimeStamp;
		}
		return tivaTimeStamp - (lastGoodP - lastGoodQ);
	}
}

long TimeSync::bootstrap(long tivaTimeStamp, long recvTimeStamp)
{
	doBootstrap = false;
	lastGoodP = tivaTimeStamp;
	lastGoodQ = recvTimeStamp;
	return recvTimeStamp;
}
