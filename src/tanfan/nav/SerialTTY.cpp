#include "tanfan/nav/SerialTTY.hpp"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <system_error>

using maav::SerialTTY;
using std::dec;
using std::hex;
using std::string;
using std::system_error;
using std::system_category;

SerialTTY::SerialTTY() : log{"SerialTTY"}
{
	connected = false;
	running = true;
}

SerialTTY::SerialTTY(const char *ttydevice) : SerialTTY() { connect(ttydevice); }
void SerialTTY::connect(const char *port)
{
	int err;

	memset(&tio, 0, sizeof(struct termios));

	tty = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_SYNC);
	if (tty == -1)
	{
		log.error() << "failed to open port " << port << " - " << strerror(errno) << commit;
		throw system_error{errno, system_category()};
	}
	else
	{
		log.debug() << "got fd " << tty << commit;
	}

	log.debug("setting up connection");
	err = tcgetattr(tty, &tio);
	if (err != 0)
	{
		log.error("could not read attributes from port");
		throw system_error{errno, system_category()};
	}

	cfsetospeed(&tio, B115200);
	cfsetispeed(&tio, B115200);

	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 100;

	tio.c_cflag &= ~CSIZE;
	tio.c_cflag |= CS8;
	tio.c_cflag &= ~PARENB;
	tio.c_cflag &= ~CSTOPB;
	tio.c_cflag &= ~CRTSCTS;
	tio.c_cflag |= CREAD;
	tio.c_cflag |= CLOCAL;

	tio.c_iflag &= ~(IXON | IXOFF | IXANY);
	tio.c_iflag &= ~IGNBRK;

	tio.c_lflag = 0;

	tio.c_oflag = 0;

	err = tcsetattr(tty, TCSANOW, &tio);
	if (err != 0)
	{
		log.error("could not set attributes on port");
		throw system_error{errno, system_category()};
	}

	log.debug("done setting up connection");
	connected = true;
}

SerialTTY::~SerialTTY() { disconnect(); }
void SerialTTY::send(const char *buffer, size_t len)
{
	int err = 0;

	log.debug() << "sending " << len << " [" << hex;
	log.debug() << static_cast<unsigned int>(buffer[0]);
	for (size_t i = 1; i < len; i++)
	{
		log.debug() << "," << static_cast<unsigned int>(buffer[i]);
	}
	log.debug() << "]" << dec << commit;

	err = write(tty, buffer, len);
	if (err > 0)
	{
		log.debug() << "sent " << err << " bytes" << commit;
	}
	else if (err == 0)
	{
		log.warn("connecting randomly closed");
	}
	else if (err < 0)
	{
		if (errno != EAGAIN && errno != EWOULDBLOCK)
		{
			log.error() << "could not send " << strerror(errno) << commit;
			throw system_error{errno, system_category()};
		}
	}
}

size_t SerialTTY::receive(char *buffer, size_t bufferLength)
{
	int err = 0;
	struct pollfd pfd;

	pfd.fd = tty;
	pfd.events = POLLIN;

	while (running)
	{
		log.debug() << "polling fd " << tty << "..." << commit;
		err = poll(&pfd, 1, 5000);

		if (err == 0)
		{
			continue;
		}
		else if (err == -1)
		{
			log.error() << "problem polling " << strerror(errno) << commit;
		}

		err = read(tty, buffer, bufferLength);
		if (err > 0)
		{
			log.debug() << "read " << err << " bytes: " << hex;
			for (int i = 0; i < err; i++)
			{
				int tmp = static_cast<unsigned int>(buffer[i]);
				tmp &= 0xFF;

				log.debug() << tmp;
			}
			log.debug() << commit;
			break;
		}
		else if (err == 0)
		{
			log.info("read nothing even though poll said I could :(");
			break;
		}
		else
		{
			log.error() << "error reading " << strerror(errno) << commit;
			throw system_error{errno, system_category()};
		}
	}

	return static_cast<size_t>(err);
}

bool SerialTTY::disconnect() noexcept
{
	if (isConnected())
	{
		connected = false;
		if (close(tty) != 0)
		{
			return false;
		}
	}

	return true;
}

bool SerialTTY::isConnected() const { return connected; }
void SerialTTY::process()
{
	running = true;
	struct pollfd pfd;

	pfd.fd = tty;
	pfd.events = POLLIN;

	while (running)
	{
		char buf[1024];

		int err = poll(&pfd, 1, 100);
		if (err < 0)
		{
			if (errno == EAGAIN || errno == EWOULDBLOCK)
			{
				continue;
			}
			log.error() << "error polling " << strerror(errno) << commit;
			throw system_error{errno, system_category()};
		}
		else if (err == 0)
		{
			continue;
		}

		err = read(tty, buf, 1024);
		if (err < 0)
		{
			log.error() << "error reading " << strerror(errno) << commit;
			throw system_error{errno, system_category()};
		}
		else if (err == 0)
		{
			log.info() << "client has closed connection" << commit;
			running = false;
			break;
		}

		cb(buf, err);
	}
}

void SerialTTY::registerCallback(void (*func)(const char *, size_t)) { cb = func; }
void SerialTTY::stop() { running = false; }
