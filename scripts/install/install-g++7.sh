#!/bin/bash
# Installs g++-7 so that std=c++17 things can be used

sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt update -y
sudo apt install gcc-7 g++-7 -y

sudo update-alternatives --remove-all gcc 
sudo update-alternatives --remove-all g++

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 1
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 1
sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 2
sudo update-alternatives --set cc /usr/bin/gcc
sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 3
sudo update-alternatives --set c++ /usr/bin/g++
