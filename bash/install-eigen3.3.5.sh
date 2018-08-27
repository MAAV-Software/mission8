#!/bin/bash

GNC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
GNC_DIR="$(dirname "$GNC_DIR")"

DESIRED_VERSION_NUMBER=3.3.5
VERSION_NUMBER=$(pkg-config --modversion eigen3)
if [ "$VERSION_NUMBER" != "$DESIRED_VERSION_NUMBER" ]; then
    echo "Installing Eigen 3.3.5"
    cd /tmp
    git clone https://github.com/eigenteam/eigen-git-mirror.git
    cd eigen-git-mirror
    git checkout 3.3.5
    mkdir build
    cd build
    cmake ..
    sudo make install
fi

cd $GNC_DIR
