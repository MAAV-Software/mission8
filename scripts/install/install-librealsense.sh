#!/bin/bash

# Get location of gnc
GNC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
GNC_DIR="$(dirname "$GNC_DIR")"

sudo apt install libusb-1.0-0-dev pkg-config -y
sudo apt-get install libglfw3-dev -y

cd /tmp/
git clone -b 'v2.19.0' https://github.com/IntelRealSense/librealsense.git
cd librealsense/
mkdir build && cd build
cmake ../

make
sudo make install

cd ../
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
sudo apt install libssl-dev -y
./scripts/patch-realsense-ubuntu-xenial.sh

echo "YEE. Have a nice day! =)"

cd $GNC_DIR
