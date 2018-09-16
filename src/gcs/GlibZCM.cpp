#include "gcs/GlibZCM.hpp"
#include "common/utils/debug.hpp"

#include <zcm/blocking.hpp>
#include <zcm/transport/udpm/udpm.hpp>

#include <iostream>

#include <stdexcept>

using namespace std;

namespace maav
{
namespace gcs
{

bool GlibZCM::handle_input()
{
	if (handle() < 0) throw runtime_error{"ZCM message handling failed!"};
	return true;
}
// Handles zcm messages when the file pointed to by the receive file descriptor (recvfd) is written
// to. This is to avoid other threads messing with gtk's main loop. In LCM, a function
// LCM::getFileno() was provided for easy access to this file descriptor, but ZCM abstracts this
// away in an attempt to be transport agnostic. The second argument to the connect() function call
// is a result of trying to get ZCM to expose its receive file descriptor.
GlibZCM::GlibZCM(const string& url) : zcm::ZCM{url}
{
	MAAV_DEBUG("Initializing ZCM");
	if (not good()) throw runtime_error{"ZCM initialization failed!"};
	zcm_connection = Glib::signal_io().connect(
		[this](Glib::IOCondition) {return handle_input();},
		((TransportUDPM*)((zcm_blocking_t*)getUnderlyingZCM()->impl)->zt)->udpm.recvfd.fd,
		Glib::IO_IN,
		Glib::PRIORITY_DEFAULT_IDLE
	);
}

GlibZCM::~GlibZCM()
{
	zcm_connection.disconnect();
}

}
}
