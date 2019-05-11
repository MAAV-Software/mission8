#!/bin/bash

# Install script install gazebo and px4 firmware for simulation use.

SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$SOFTWARE_DIR")"

git submodule update --init --recursive

# Install gazebo9 simulator
echo "Installing Gazebo9"
sleep 3

sudo apt install protobuf-compiler -y
curl -sSL http://get.gazebosim.org | sh

# Install PX4 firmware
echo "Installing PX4 firmware"
sleep 3

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

# Ubuntu Config
echo "We must first remove modemmanager"
sudo apt remove modemmanager -y

# Common dependencies
echo "Installing common dependencies"
sudo apt update -y
sudo apt install git zip qtcreator cmake build-essential genromfs ninja-build libopencv-dev -y
# Required python packages
sudo apt install python-argparse python-empy python-toml python-numpy python-dev python-pip python-yaml -y
sudo -H pip install --upgrade pip
sudo -H pip install pandas jinja2 pyserial
# optional python tools
sudo -H pip install pyulog
sudo -H pip install pyyaml

# Install FastRTPS 1.5.0 and FastCDR-1.0.7
fastrtps_dir=$HOME/eProsima_FastRTPS-1.5.0-Linux
echo "Installing FastRTPS to: $fastrtps_dir"
if [ -d "$fastrtps_dir" ]
then
    echo " FastRTPS already installed."
else
    pushd .
    cd ~
    wget http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-5-0/eprosima_fastrtps-1-5-0-linux-tar-gz -O eprosima_fastrtps-1-5-0-linux.tar.gz
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz eProsima_FastRTPS-1.5.0-Linux/
    tar -xzf eprosima_fastrtps-1-5-0-linux.tar.gz requiredcomponents
    tar -xzf requiredcomponents/eProsima_FastCDR-1.0.7-Linux.tar.gz
    cpucores=$(( $(lscpu | grep Core.*per.*socket | awk -F: '{print $2}') * $(lscpu | grep Socket\(s\) | awk -F: '{print $2}') ))
    cd eProsima_FastCDR-1.0.7-Linux; ./configure --libdir=/usr/lib; make -j$cpucores; sudo make install
    cd ..
    cd eProsima_FastRTPS-1.5.0-Linux; ./configure --libdir=/usr/lib; make -j$cpucores; sudo make install
    cd ..
    rm -rf requiredcomponents eprosima_fastrtps-1-5-0-linux.tar.gz
    popd
fi


# Download QGroundControl
if [ ! -f exe/QGroundControl.AppImage ]; then
	echo "Downloading QGroundControl"
	mkdir exe
	cd exe
	wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/builds/master/QGroundControl.AppImage
	chmod a+x QGroundControl.AppImage
	cd ..
fi

sudo usermod -a -G dialout $USER
sudo apt remove modemmanager -y

# Build and run simulator for the first time
pwd
cd ${SOFTWARE_DIR}/thirdparty/Firmware
num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make posix_sitl_default gazebo -j$((num_procs_avail > 1 ? num_procs_avail : 1))
