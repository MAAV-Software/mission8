#!/bin/bash
pushd ~
sudo apt install libglib2.0-dev
wget https://github.com/lcm-proj/lcm/releases/download/v1.3.1/lcm-1.3.1.zip
unzip lcm-1.3.1.zip
rm lcm-1.3.1.zip
pushd lcm-1.3.1
./configure
make
sudo make install
sudo ldconfig
popd
rm -rf lcm-1.3.1

