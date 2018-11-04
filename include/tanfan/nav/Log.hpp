#ifndef MAAV_LOG_HPP
#define MAAV_LOG_HPP

// some useful standard-library stuff
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <type_traits>

// use the MAAV namespace
namespace maav
{
/**
 * @brief Manages a log
 * @author Daniel Woodworth (dascwo)
 *
 * @details A Log can be used to write logging messages to the console
 * (clog/stderr) and to a certain logging file specified during initialization.
 * Log consists entirely of static members and there is no need to construct
 * a Log object. However, Log must be initialized exactly once by calling
 * init(const char*, Level, Level) before using any of the other logging
 * functions. There are four different logging levels that can be used
 * with Log: debug, info, warn, and error. When initializing Log, one can
 * set up message filtering by specifying a minimum log level for the console
 * and file logging separately (the order is the same listed before, and in
 * Log::Level).
 *
 * A message can be logged using the four log-level-specific static functions
 * provided by log:
 * info(const char*, const std::string&, std::chrono::system_clock::time_point)
 * ,
 * error(const char*, const std::string&,std::chrono::system_clock::time_point)
 * ,
 * warn(const char*, const std::string&, std::chrono::system_clock::time_point)
 * , and
 * debug(const char*, const std::string&,std::chrono::system_clock::time_point)
 * . Each of these functions takes a "module identifier" (a short string used
 * to identify different parts of the code in the log output), a timestamp
 * (which conveniently defaults to the current time), and the log message to
 * write as a std::string. Here's an example of these methods' use:
 *
 *     maav::Log::info("doc", "info logging example");
 *     maav::Log::error("doc", "here's an error test");
 *     maav::Log::warn("doc", "this warning happened five seconds ago",
 *             std::chrono::system_clock::now() - std::chrono::seconds{5});
 *
 * These four methods can be used directly (and they may be useful if one
 * single message needs to be logged in a part of the program), but it is
 * recommended that logging be done using the interface of the nested class
 * Logger for most cases. A Logger object should be used only in a single scope
 * (/function), and it will keep track of a "module identifier" for that scope
 * (which is specified at construction, preferably as a string literal). It
 * also supports analogous member functions to Log's static functions, namely:
 * Logger::info(const std::string&, std::chrono::system_clock::time_point),
 * Logger::error(const std::string&, std::chrono::system_clock::time_point),
 * Logger::warn(const std::string&, std::chrono::system_clock::time_point), and
 * Logger::debug(const std::string&, std::chrono::system_clock::time_point).
 * The following code is essentially equivalent to that given above:
 *
 *     maav::Log::Logger logger {"doc"};
 *     logger.info("info logging example");
 *     logger.error("here's an error test");
 *     logger.warn("this warning happened five seconds ago",
 *             std::chrono::system_clock::now() - std::chrono::seconds{5});
 *
 * Besides this interface, Logger also supports an ostream interface that can
 * be used to more easily log values such as numbers without having to write
 * to strings and concatenate them together. This is supported by the
 * no-parameter overloads of the logging functions,
 * Logger::info(), Logger::error(), Logger::warn(), and Logger::debug(),
 * which are used to switch the logging level of the stream, and by the
 * provided stream manipulator commit(std::ostream&), which writes the current
 * contents of the stream into the log at the current level. The following is
 * an example of how this method may be used:
 *
 *     maav::Log::Logger logger {"doc"};
 *     logger.info() << "this is an info message" << maav::commit;
 *     logger.error() << "this is an error" << maav::commit;
 *     logger.debug() << "x = " << x << maav::commit;
 *     logger.info() << "chosen waypoint: " << waypoint << maav::commit;
 *
 * A final note on thread safety: all of Log's methods are thread-safe (log
 * messages can be written from multiple threads simultaneously without
 * resulting in race conditions or getting interleaved), but the Logger
 * inteface is not entirely: a Logger object is meant to be used in only
 * one scope and on only one thread. Multiple Loggers may be used
 * simultaneously, but one Logger should not be used simultaneously on
 * different threads.
 */
class Log
{
   public:
    /**
     * @brief A type for representing a log level
     */
    enum class Level
    {
        debug,
        info,
        warn,
        error
    };

   private:
    // mutexes for the streams
    static std::mutex clog_mutex, file_mutex;

    // the file stream
    static std::ofstream file_stream;

    // the minimum logging levels for clog and for the file
    static Level clog_level, file_level;

    /**
     * @brief A helper type that takes care of time formatting for
     * the different logging functions
     * @author Daniel Woodworth (dascwo)
     */
    class TimeWriter
    {
        // keep track of a tm and a millisecond measure
        std::tm time_struct;
        std::chrono::milliseconds millis;

       public:
        /**
         * @brief Constructs a TimeWriter with a certain time
         */
        explicit TimeWriter(std::chrono::system_clock::time_point);

        /**
         * @brief Writes the time
         */
        friend std::ostream& operator<<(std::ostream&, const TimeWriter&);
    };

    // we need to friend TimeWriter's operator<< out here too
    friend std::ostream& operator<<(std::ostream&, const TimeWriter&);

   public:
    /**
     * @brief No instantiation for you
     */
    Log() = delete;

    /**
     * @brief Initializes the log and sets the filename
     * @param file_name The file to log into
     * @param file_level The minimum log level for the file
     * @param clog_level The minimum log level for clog
     */
    static void init(const char* file_name = "maav.log", Level file_level = Level::debug,
        Level clog_level = Level::info);

    /**
     * @brief Writes an info-level log entry
     * @param mod_id The "module identifier" that is printed
     * at the start of the line
     * @param message The log message to display
     * @param timestamp The time to record with this entry
     */
    static void info(const char* mod_id, const std::string& message,
        std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now());

    /**
     * @brief Writes an error-level log entry
     * @param mod_id The "module identifier" that is printed
     * at the start of the line
     * @param message The log message to display
     * @param timestamp The time to record with this entry
     */
    static void error(const char* mod_id, const std::string& message,
        std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now());

    /**
     * @brief Writes a warning-level log entry
     * @param mod_id The "module identifier" that is printed
     * at the start of the line
     * @param message The log message to display
     * @param timestamp The time to record with this entry
     */
    static void warn(const char* mod_id, const std::string& message,
        std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now());

    /**
     * @brief Writes a debug-level log entry
     * @param mod_id The "module identifier" that is printed
     * at the start of the line
     * @param message The log message to display
     * @param timestamp The time to record with this entry
     */
    static void debug(const char* mod_id, const std::string& message,
        std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now());

    /**
     * @brief A local logger stream to be used with Log
     * @author Daniel Woodworth (dascwo)
     * @see Log
     */
    class Logger : public std::ostringstream
    {
        // our "module identifier"
        const char* mod_id;

        // the log level that the logger is set to
        //(info seems like a sensible default)
        Level level{Level::info};

       public:
        /**
         * @brief Constructs a Logger
         * @param mod_id The "module identifier" to use
         */
        explicit Logger(const char* mod_id_in) : mod_id{mod_id_in} {}
        /**
         * @brief Writes an info-level log entry
         * @param message The log message to display
         * @param timestamp The time to record with this entry
         */
        void info(const std::string& message,
            std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now())
        {
            Log::info(mod_id, message, timestamp);
        }

        /**
         * @brief Writes an error-level log entry
         * @param message The log message to display
         * @param timestamp The time to record with this entry
         */
        void error(const std::string& message,
            std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now())
        {
            Log::error(mod_id, message, timestamp);
        }

        /**
         * @brief Writes a warning-level log entry
         * @param message The log message to display
         * @param timestamp The time to record with this entry
         */
        void warn(const std::string& message,
            std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now())
        {
            Log::warn(mod_id, message, timestamp);
        }

        /**
         * @brief Writes a debug-level log entry
         * @param message The log message to display
         * @param timestamp The time to record with this entry
         */
        void debug(const std::string& message,
            std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now())
        {
            Log::debug(mod_id, message, timestamp);
        }

        /**
         * @brief Sets the logger to the info log level
         * and returns a reference so it can be used as such
         */
        Logger& info()
        {
            level = Level::info;
            return *this;
        }

        /**
         * @brief Sets the logger to the error log level
         * and returns a reference so it can be used as such
         */
        Logger& error()
        {
            level = Level::error;
            return *this;
        }

        /**
         * @brief Sets the logger to the warning log level
         * and returns a reference so it can be used as such
         */
        Logger& warn()
        {
            level = Level::warn;
            return *this;
        }

        /**
         * @brief Sets the logger to the debug log level
         * and returns a reference so it can be used as such
         */
        Logger& debug()
        {
            level = Level::debug;
            return *this;
        }

        /**
         * @brief "Commits" a Logger, writing its current contents to
         * its log at the correct log level and the clearing it
         * @see commit(std::ostream&)
         */
        void commit()
        {
            // switch according to the log level
            switch (level)
            {
                case Level::info:
                    info(std::ostringstream::str());
                    break;
                case Level::error:
                    error(std::ostringstream::str());
                    break;
                case Level::warn:
                    warn(std::ostringstream::str());
                    break;
                case Level::debug:
                    debug(std::ostringstream::str());
                    break;
            }

            // and then clear the string
            std::ostringstream::str("");
        }
    };
};

/**
 * @brief An I/O manipulator for "committing" streams
 * @author Daniel Woodworth (dascwo)
 *
 * @details If the stream is a Log::Logger, it will have commit called on it;
 * otherwise, undefined behavior will ensue
 */
std::ostream& commit(std::ostream&);
}

#endif  // MAAV_LOG_HPP
