#!/bin/bash
# Installs a newer version of CMake than shipped by Ubuntu
# Mainly for Ubuntu users
sudo apt-get remove cmake -y
sudo apt install -y libncurses5-dev
version=3.10
build=3
mkdir /tmp/install-cmake
cd /tmp/install-cmake
wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
tar -xzvf cmake-$version.$build.tar.gz
cd cmake-$version.$build/
./configure --prefix=/usr
./bootstrap --system-curl
make -j4
sudo make install
