#!/bin/bash
#
# Install OpenCV 3 on your machine

sudo apt install ffmpeg -y

cd /tmp
git clone -b 3.4 https://github.com/opencv/opencv.git
git clone -b 3.4 https://github.com/opencv/opencv_contrib.git
mv opencv_contrib/modules/dnn_modern/CMakeLists.txt \
   opencv_contrib/modules/dnn_modern/CMakeLists.txt.bak

cd opencv
mkdir build
cd build

cmake -D BUILD_PNG=ON \
      -D BUILD_JPEG=ON \
      -D BUILD_ZLIB=ON \
      -D OPENCV_CXX11=ON \
      -D OPENCV_EXTRA_MODULES_PATH='/tmp/opencv_contrib/modules' \
      -D CMAKE_BUILD_TYPE='Release' \
      ..

num_procs_avail=$(($(grep -c ^processor /proc/cpuinfo)-1))
make -j$((num_procs_avail > 1 ? num_procs_avail : 1))

sudo apt remove libopencv* -y

sudo make install
