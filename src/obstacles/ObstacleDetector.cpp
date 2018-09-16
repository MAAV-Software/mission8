#include "obstacles/ObstacleDetector.hpp"

#include <algorithm>
#include <functional>
#include <cmath>

using maav::ObstacleDetector;
using std::sort;
using std::vector;

ObstacleDetector::ObstacleDetector()
{
}

ObstacleDetector::ObstacleDetector(float startAngle, float angleStep) :
	start(startAngle), step(angleStep)
{
}

vector<ObstacleDetector::obstacle_data> ObstacleDetector::detect(
		const vector<float>& laser_data)
{
	//a function object that determines if things are "far"
	auto is_far = [this](float d) {return d < far_margin;};

	//and if they are not "far," since not1 doesn't actually do that
	auto not_far = [&is_far](float d) {return not is_far(d);};

	//make a vector to store the obstacle data
	vector<obstacle_data> datas;

	//start at the beginning
	auto region_start = begin(laser_data);

	//repeatedly locate the next non-"far" value
	//stop if none exists
	while ((region_start = find_if(region_start, end(laser_data),
					not_far)) != end(laser_data))
	{
		//find the next "far" value; that's the end of the region
		auto region_end = find_if(region_start, end(laser_data), is_far);

		//cull one-point regions
		if (next(region_start) == region_end)
		{
			region_start = region_end;
			continue;
		}

		//go through non-jump regions
		auto obst_start = region_start, obst_end = next(obst_start);
		while (obst_start != region_end)
		{
			while (obst_end != region_end
					and abs(obst_end[0] - obst_end[-1]) < jump_margin) ++obst_end;

			//find the size of the current non-jump region
			auto obst_size = distance(obst_start, obst_end);

			//find the midpoint of the obstacle
			float mid_dist {(obst_size & 1) ? obst_start[obst_size/2]
				: (obst_start[obst_size/2 - 1] + obst_start[obst_size/2])/2.0};
			float mid_theta {start + distance(begin(laser_data), obst_start)*step
				+ (obst_size - 1)*step/2.0};

			//calculate the width
			float width {(*obst_start + obst_end[-1])
				* sin((obst_size - 1)*step/2.0)};

			//insert into datas
			datas.push_back({{mid_dist*cos(mid_theta), mid_dist*sin(mid_theta)},
					width});

			//move to the next one
			obst_start = obst_end++;
		}

		//start the next region after this one
		region_start = region_end;
	}

	//return the data on the obstacles found
	return datas;
}
