#ifndef LASERREADER_HPP
#define LASERREADER_HPP

#include <string>
#include <vector>
#include "common/utils/SerialTTY.hpp"

namespace maav
{
/**
 * @brief Shared implementation functionality for reading laser data
 *
 * @details As this doesn't currently have any data members, private inheritance
 * is the suggested way to use it in order to take advantage of the empty base
 * optimization
 */
class LaserReaderDecoder
{
public:
    /**
     * @brief Decodes laser reader data to produce distance values
     * @param data The data to read
     * @return The distances at each measurement (in meters)
     */
    std::vector<double> getDistances(const std::string& data);

    // a helper function; not really part of the public interface, but needs to
    // be exposed for testing
    int decode(const std::string& data, int len);
};

/**
 * @brief A class to read from a URG laser
 * @author Luke Krasner (luke@lukekrasner.com)
 *
 * @details Contains functionality to read from a serial port and to transform
 * the data that it receives. All functions that transform raw data to the
 * variety of useful data can be called by passing in raw data.
 */
class SerialLaserReader : LaserReaderDecoder
{
public:
    /**
     * @brief Opens the given port for communication
     * @param serialPort Path of serial port to connect to
     */
    explicit SerialLaserReader(const char* serialPort);

    /**
     *@brief shuts down laser and closes serial port
     */
    ~SerialLaserReader();

    /**
     * @brief Starts the laser allowing measurement to begin
     */
    bool connect(const char* serialPort);

    /**
     * @brief Stops the laser
     */
    bool disconnect();

    /**
     * @brief Checks to see if currently open
     */
    bool isConnected() const;

    /**
     * @brief sets the laser to SCIP2.0 mode. Likely not needed
     */
    bool setSCIP2();

    /**
     * @brief Returns the range of the laser in meters
     * @return The range
     */
    double getRange() const noexcept;

    /**
     * @brief get basic version and product info from the laser
     */
    std::string getInfo();

    /**
     * @brief actually gets data and returns a map of the angles and distances
     * @return vector of distances (in meters)
     */
    std::vector<double> getDistances();

    /**
     * @brief Retrieves the start angle property from the laser
     * @return The start angle
     */
    double getStartAngle();

    /**
     * @brief Retrieves the angle step size property from the laser
     * @return The angular step size
     */
    double getStep();

private:
    std::string send(const std::string& command);

    std::string getDistData(int start, int end, int resolution);

    std::string zeroPad(int number, int len);

    SerialTTY tty;
};

/**
 * @brief A type for reading provided laser data directly for
 * testing/visualizing purposes.
 */
class StaticLaserReader : LaserReaderDecoder
{
public:
    // LaserReaderDecoder's getDistances is sufficient for this class
    using LaserReaderDecoder::getDistances;

    // again for testing purposes
    using LaserReaderDecoder::decode;

    /**
     * @brief Returns the range of the laser in meters
     * @return The range
     */
    double getRange() const noexcept;

    /**
     * @brief Retrieves the start angle property from the laser
     * @return The start angle
     */
    double getStartAngle();

    /**
     * @brief Retrieves the angle step size property from the laser
     * @return The angular step size
     */
    double getStep();
};
}

#endif
