#!/bin/bash
#
# Generates the zcm messages

#Script expects to be in software/scripts/tools
SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && cd ../.. && pwd )"
GENERATED_DIR="${SOFTWARE_DIR}/generated"
MAVLINK_DIR="${SOFTWARE_DIR}/thirdparty/mavlink"
LANG=C

# Delete Generated messages
rm -r ${GENERATED_DIR}/mavlink

# Install dependencies
sudo apt install -y python3-pip
pip install --user future
sudo apt install -y python-tk
#check that mavlink is installed? it should be installed in setup script?
PYTHONPATH=${SOFTWARE_DIR}/thirdparty/mavlink

cd ${MAVLINK_DIR}

# Generate messages
python -m pymavlink.tools.mavgen --lang=${LANG} --wire-protocol=2.0 \
	--output=${GENERATED_DIR}/mavlink/v2.0 \
	${MAVLINK_DIR}/message_definitions/v1.0/common.xml
