#!/bin/bash

# Get location of software
SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$SOFTWARE_DIR")"

sudo cp ${SOFTWARE_DIR}/scripts/99-pixhawk.rules /etc/udev/rules.d
sudo cp ${SOFTWARE_DIR}/scripts/99-microstrain-libusb.rules /etc/udev/rules.d