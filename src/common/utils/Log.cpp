
//include the declarations
#include "common/utils/Log.hpp"

//and some more time-printing-related headers
#include <ctime>
#include <iomanip>

#include <cassert>

//use things in std
using namespace std;

//and use std::chrono
using namespace std::chrono;

//reopen the maav namespace
namespace maav
{

//open up an anonymous namespace for defining put_time
//this can be removed when GCC actually implements it
namespace
{

/**
 * @brief a helper struct for put_time<CharT>(const std::tm*, const CharT*)
 * @author Daniel Woodworth (dascwo)
 * @see put_time<CharT>(const std::tm*, const CharT*)
 */
template <typename CharT>
struct put_time_holder
{
	const std::tm* time;
	const CharT* format;
};

/**
 * @brief An implementation of
 * <a href="http://en.cppreference.com/w/cpp/io/manip/put_time">
 * std::put_time</a>
 * @author Daniel Woodworth (dascwo)
 *
 * @details The code for this is largely adapted from the code found <a
 * href="http://en.cppreference.com/w/cpp/io/manip/put_time#Return_value">
 * here</a>
 */
template <typename CharT>
put_time_holder<CharT> put_time(const std::tm* time, const CharT* format)
{
	return {time, format};
}

/**
 * @brief The actual guts of put_time(const std::tm*, const CharT*)
 * @author Daniel Woodworth (dascwo)
 * @see put_time(const std::tm*, const CharT*)
 */
template <typename CharT, typename Traits>
std::basic_ostream<CharT, Traits>& operator<< (
	std::basic_ostream<CharT, Traits>& out,
	const put_time_holder<CharT>& holder)
{
	//make a sentry to make sure everything stays okay
	typename std::basic_ostream<CharT, Traits>::sentry sentry {out};

	//a nickname for std::ostreambuf_iterator<CharT, Traits>
	//the following code is bad enough as is
	using osb_iterator = std::ostreambuf_iterator<CharT, Traits>;

	//apply the time_put facet to our time
	auto out_iter = std::use_facet<std::time_put<CharT, osb_iterator>>
		(out.getloc()).put(
		osb_iterator{out}, out, out.fill(), holder.time, holder.format,
		holder.format + Traits::length(holder.format));

	//if the write failed, update the stream
	if (out_iter.failed()) out.setstate(std::ios_base::badbit);

	//return the stream
	return out;
}

}

//implement time writing
ostream& operator<< (ostream& out, const Log::TimeWriter& tw)
{
	return out << put_time(&tw.time_struct, "%FT%T") << "."
		<< std::setfill('0') << std::setw(3) << tw.millis.count();
}

//instantiate the static members here
mutex Log::clog_mutex, Log::file_mutex;
ofstream Log::file_stream;
Log::Level Log::clog_level, Log::file_level;

//implement the log's file stream creation
void Log::init(const char* file_name, Level file_level_in, Level clog_level_in)
{
	//set the log levels appropriately
	file_level = file_level_in, clog_level = clog_level_in;

	//open the requested log file for appending
	file_stream.open(file_name, ios_base::app);

	//figure out the current time
	std::tm time_struct;
	std::time_t time {system_clock::to_time_t(system_clock::now())};
	localtime_r(&time, &time_struct);

	//write the message to the file
	{
		std::lock_guard<std::mutex> lock {file_mutex};
		assert(file_stream);
		file_stream << "-------- " << put_time(&time_struct, "%c")
			<< " --------" << endl;
	}
}

//implement the time writing pre-calculations
Log::TimeWriter::TimeWriter(system_clock::time_point time)
{
	//this code adapts a method for printing more precise times
	//discussed here:
	//http://stackoverflow.com/questions/15845505/how-to-get-higher-precision-fractions-of-a-second-in-a-printout-of-current-tim

	//get the time of the last full second
	system_clock::time_point last_second {duration_cast<seconds>(
		time.time_since_epoch())};

	//set our tm
	std::time_t time_t_time {system_clock::to_time_t(last_second)};
	localtime_r(&time_t_time, &time_struct);

	//and get the number of milliseconds
	millis = duration_cast<milliseconds>(time - last_second);
}

//implement writing info messages
void Log::info(const char* mod_id, const std::string& message,
	std::chrono::system_clock::time_point timestamp)
{
	TimeWriter tw {timestamp};
	if (clog_level <= Level::info) {
		std::lock_guard<std::mutex> lock {clog_mutex};
		clog << "INFO  " << tw << " " << mod_id << ": " << message
			<< endl;
	}
	if (file_level <= Level::info) {
		std::lock_guard<std::mutex> lock {file_mutex};
		assert(file_stream);
		file_stream << "INFO  " << tw << " " << mod_id << ": "
			<< message << endl;
	}
}

//and error messages
void Log::error(const char* mod_id, const std::string& message,
	std::chrono::system_clock::time_point timestamp)
{
	TimeWriter tw {timestamp};
	if (clog_level <= Level::error) {
		std::lock_guard<std::mutex> lock {clog_mutex};
		clog << "\033[1;31mERROR\033[0m " << tw << " " << mod_id
			<< ": " << message << endl;
	}
	if (file_level <= Level::error) {
		std::lock_guard<std::mutex> lock {file_mutex};
		assert(file_stream);
		file_stream << "ERROR " << tw << " " << mod_id << ": "
			<< message << endl;
	}
}

//and warning messages
void Log::warn(const char* mod_id, const std::string& message,
	std::chrono::system_clock::time_point timestamp)
{
	TimeWriter tw {timestamp};
	if (clog_level <= Level::warn) {
		std::lock_guard<std::mutex> lock {clog_mutex};
		clog << "\033[34mWARN\033[0m  " << tw << " " << mod_id
			<< ": " << message << endl;
	}
	if (file_level <= Level::warn) {
		std::lock_guard<std::mutex> lock {file_mutex};
		assert(file_stream);
		file_stream << "WARN  " << tw << " " << mod_id << ": "
			<< message << endl;
	}
}

//and debug messages
void Log::debug(const char* mod_id, const std::string& message,
	std::chrono::system_clock::time_point timestamp)
{
	TimeWriter tw {timestamp};
	if (clog_level <= Level::debug) {
		std::lock_guard<std::mutex> lock {clog_mutex};
		clog << "\033[33mDEBUG\033[0m " << tw << " " << mod_id << ": "
			<< message << endl;
	}
	if (file_level <= Level::debug) {
		std::lock_guard<std::mutex> lock {file_mutex};
		assert(file_stream);
		file_stream << "DEBUG " << tw << " " << mod_id << ": "
			<< message << endl;
	}
}

//implement the I/O manipulator
ostream& commit(ostream& stream)
{
	//use dynamic_cast to try to get a Logger pointer
	//no, use reinterpret_cast instead since we don't have RTTI
	if (Log::Logger* logger = reinterpret_cast<Log::Logger*>(&stream))
		logger->commit();

	//return the stream
	return stream;
}

}
