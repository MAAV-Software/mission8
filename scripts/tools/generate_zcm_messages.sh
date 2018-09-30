#!/bin/bash
#
# Generates the zcm messages

SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$GNC_DIR")"
GENERATED_DIR="${SOFTWARE_DIR}/generated/common/messages"
ZCM_TYPES_DIR="${SOFTWARE_DIR}/msgtypes/zcm"

# Delete Generated messages
find ${SOFTWARE_DIR}/generated/common/messages -name ".hpp" -type f -delete 

zcm-gen --cpp --cpp-hpath ${GENERATED_DIR} ${ZCM_TYPES_DIR}/*.zcm