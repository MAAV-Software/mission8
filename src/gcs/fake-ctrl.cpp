#include "gcs/zcm_type_ops.hpp"
#include "common/messages/MsgChannels.hpp"

#include <zcm/zcm-cpp.hpp>

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std;
using namespace maav;
using namespace std::literals::chrono_literals;

using params_t = ctrl_params_t;

ostream& operator<<(ostream& out, const pid_gains_t& p)
{
	return out << "\t" << p.p << "\t" << p.i << "\t" << p.d << "\n";
}

ostream& operator<<(ostream& out, const params_t& cp)
{
	out << "value:\n";
	for (const auto& v : cp.value) out << v;
	out << "rate:\n";
	for (const auto& r : cp.rate) out << r;
	return out;
}

ostream& operator<<(ostream& out, const waypoint_t& sp)
{
	return out << sp.pose[0] << " " << sp.pose[1] << " " << sp.pose[2] << " " << sp.pose[3];
}

class FakeCtrl
{
	zcm::ZCM zcm;

	params_t params;
	waypoint_t waypoint;

	mutex lock;

	void handle_params(const zcm::ReceiveBuffer*, const string&,
		const params_t* msg)
	{
		lock_guard<mutex> g {lock};
		params = *msg;
		cout << "new params:\n" << params << endl;
	}

	//state transitions are handled here
	void handle_waypoint(const zcm::ReceiveBuffer*, const string&,
		const waypoint_t* msg)
	{
		{
		lock_guard<mutex> g {lock};
		waypoint = *msg;
		}
		if (waypoint.pmode == 0) cout << "autonomous mode!" << endl;
		if (waypoint.pmode == 1) cout << "hovering!" << endl;
		if (waypoint.pmode == 2) cout << "taking off!" << endl;
		if (waypoint.pmode == 3) cout << "landing!" << endl;
		if (msg->pmode == 4 and (msg->mode == 1 or msg->mode == 3) and *msg != waypoint)
			cout << "going to pose " << *msg << endl;
		if (msg->pmode == 4 and (msg->mode == 2 or msg->mode == 3) and *msg != waypoint)
			cout << "setting rate to " << *msg << endl;
		if (waypoint.pmode == 5) cout << "leaving arena!" << endl;
	}

public:

	FakeCtrl() : zcm{"ipc"}
	{
		//the fake controller starts with all parameters set to 0 and landed
		pid_gains_t pid;
		pid.p = 0;
		pid.i = 0;
		pid.d = 0;
		fill(begin(params.value), end(params.value), pid);
		fill(begin(params.rate), end(params.rate), pid);
		waypoint.mode = 0;
		fill(begin(waypoint.pose), end(waypoint.pose), 0);
		cout << "starting params:\n" << params << endl;
		cout << "vehicle is landed" << endl;

		zcm.subscribe(CTRL_PARAMS_CHANNEL, &FakeCtrl::handle_params, this);
		zcm.subscribe(PLANNER_CMD, &FakeCtrl::handle_waypoint, this);

		cout << "starting params:\n" << params << endl;
		cout << "vehicle is landed" << endl;

		//this thread handles periodic status broadcasts
		thread t {[this]() {
			while (true) {
				{
					lock_guard<mutex> g{lock};
					zcm.publish(CTRL_HEARTBEAT_CHANNEL, &params);
					zcm.publish(PLANNER_STAT, &waypoint);
				}
				this_thread::sleep_for(500ms);
			}
		}};

		if (not zcm.good()) throw runtime_error{"ZCM initialization failed!"};

		zcm.run();

		t.join();
	}
};

int main()
{
	cout << "START PROGRAM" << endl;
	FakeCtrl c{};
}
