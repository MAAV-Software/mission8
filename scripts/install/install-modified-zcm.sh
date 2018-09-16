#!/bin/bash
#
# Install the MAAV modified ZCM on your machine
# Author: Romario Pashollari (rpash)
#         Cheng Jiang (chengjia)

# Set JAVA_HOME path
grep -q JAVA_HOME ~/.bashrc /etc/environment || \
    export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
export PATH=$JAVA_HOME/bin:$PATH

#Download modified zcm repo
cd /tmp
git clone git@gitlab.eecs.umich.edu:maav/maav-zcm.git
cd maav-zcm

mkdir -p zcm/transport/third-party

# Install modified ZCM
./scripts/install-deps.sh
./waf configure --use-all --use-third-party
./waf build
sudo ./waf install

# Set JAVA_HOME, PATH, LD_LIBRARY_PATH in .bashrc
grep -q JAVA_HOME ~/.bashrc /etc/environment || \
    echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc
grep -q 'export PATH=$JAVA_HOME/bin:$PATH' ~/.bashrc /etc/environment || \
    echo 'export PATH=$JAVA_HOME/bin:$PATH' >> ~/.bashrc
grep -q 'export LD_LIBRARY_PATH=/usr/lib:/usr/local/lib' ~/.bashrc /etc/environment || \
    echo 'export LD_LIBRARY_PATH=/usr/lib:/usr/local/lib' >> ~/.bashrc
source ~/.bashrc
