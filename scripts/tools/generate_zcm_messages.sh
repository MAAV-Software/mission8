#!/bin/bash
#
# Generates the zcm messages
SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$SOFTWARE_DIR")"
SOFTWARE_DIR="${SOFTWARE_DIR}/.."
GENERATED_DIR="${SOFTWARE_DIR}/generated/common/messages"
ZCM_TYPES_DIR="${SOFTWARE_DIR}/msgtypes/zcm"

echo ${SOFTWARE_DIR}

# Delete Generated messages
rm -r ${SOFTWARE_DIR}/generated

zcm-gen --cpp --cpp-hpath ${GENERATED_DIR} ${ZCM_TYPES_DIR}/*.zcm
