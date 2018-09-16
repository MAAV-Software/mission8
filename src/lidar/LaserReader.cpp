#include "common/math/math.hpp"
#include "lidar/LaserReader.hpp"
#include <boost/algorithm/string/find.hpp>
#include <iomanip>
#include <sstream>

using boost::find_nth;
using boost::iterator_range;
using std::make_pair;
using std::pair;
using std::setfill;
using std::setw;
using std::string;
using std::stringstream;
using std::vector;

namespace maav
{

vector<double> LaserReaderDecoder::getDistances(const string& data_orig)
{
	auto data = data_orig;
	vector <double> dist;
	size_t pos = 0;

	auto range = find_nth(data, "\n", 2);
	data.erase(0, distance(data.begin(), range.end()));
	data.erase(data.length() - 3);

	while((pos = data.find("\n", pos)) != string::npos)
	{
		data.erase(pos - 1, 2);
	}

	for(size_t i = 0; i < data.length() - 3; i += 3)
	{
		dist.push_back(decode(data.substr(i, 3), 3) / 1000.0);
	}

	return dist;
}

int LaserReaderDecoder::decode(const string& data, int len)
{
	int value = 0;
	for(int i = 0; i < len; i++)
	{
		value <<= 6;
		int v = (data[i]-0x30)&0x3f;
		value += v;
	}

	return value;
}

SerialLaserReader::SerialLaserReader(const char *serialPort)
{
	connect(serialPort);
}

SerialLaserReader::~SerialLaserReader()
{
	disconnect();
}

string SerialLaserReader::send(const string& command)
{
	char buf[2200];
	size_t len;
	string response;

	memset((void*) buf, '\0', 2200);
	tty.send(command.c_str(), command.length());

	len = tty.receive(buf, 2200);

	for (size_t i = 0; i < len; i++)
	{
		response.push_back(buf[i]);
	}

	return response;
}

string SerialLaserReader::getInfo()
{
	string version = send("VV\n");
	string specs = send("PP\n");
	return version + specs;
}

double SerialLaserReader::getStep()
{
	return 0.35139 * PI / 180.0;
}

double SerialLaserReader::getStartAngle()
{
	return -30.0 * PI / 180.0;
}

bool SerialLaserReader::setSCIP2()
{
	string response = send("SCIP2.0\n");
	if(response != "SCIP2.0\n0\n\n")
		return false;
	return true;
}

bool SerialLaserReader::connect(const char *serialPort)
{
	string response;

	if (tty.isConnected())
	{
		return true;
	}

	tty.connect(serialPort);

	response = send("BM\n");

	if(response == "BM\n00P\n\n" || response == "BM\n02R\n\n")
	{
		return true;
	}

	return false;
}

bool SerialLaserReader::disconnect()
{
	if (!isConnected())
	{
		return true;
	}

	string response = send("QT\n");

	bool ok = (response == "QT\n00P\n\n");

	tty.disconnect();

	return ok;
}

string SerialLaserReader::zeroPad(int number, int len)
{
	stringstream ss;
	ss << setw(len) << setfill('0') << number;
	return ss.str();
}

string SerialLaserReader::getDistData(int start, int end, int resolution)
{
	string command;
	command = "GD" + zeroPad(start, 4) + zeroPad(end, 4) + zeroPad(resolution, 2) + "\n";
	return send(command);
}

bool SerialLaserReader::isConnected() const
{
	return tty.isConnected();
}

vector<double> SerialLaserReader::getDistances()
{
	string distData;

	// startLaser();

	int RETRIES = 3;

	for(int i = 0; i < RETRIES; i++)
	{
		distData = getDistData(44, 725, 1);

		if(distData.length() == 2134)
		{
			return LaserReaderDecoder::getDistances(distData);
		}
	}

	return vector<double>();
}

// TODO get this from the SCIP 2.0 protocol or something
#define URG_04LX_UG01_RANGE 4.0
double SerialLaserReader::getRange() const noexcept
{
	return URG_04LX_UG01_RANGE;
}

//the static LaserReader type just uses constants for all of the query methods
double StaticLaserReader::getRange() const noexcept
{
	return URG_04LX_UG01_RANGE;
}
double StaticLaserReader::getStartAngle()
{
	return -30.0 * PI / 180.0;
}
double StaticLaserReader::getStep()
{
	return 0.35139 * PI / 180.0;
}

}
