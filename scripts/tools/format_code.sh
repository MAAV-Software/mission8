#!/bin/bash
#
# Formats all code with clang-format

SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$GNC_DIR")"
SRC_DIR="${SOFTWARE_DIR}/src"
INCLUDE_DIR="${SOFTWARE_DIR}/include"
TEST_DIR="${SOFTWARE_DIR}/tests"
DRIVERS_DIR="${SOFTWARE_DIR}/drivers"

# Delete Generated messages
find ${SRC_DIR} -iname "*.cpp" | xargs clang-format -i -style=file
find ${TEST_DIR} -iname "*.cpp" | xargs clang-format -i -style=file
find ${DRIVERS_DIR} -iname "*.cpp" | xargs clang-format -i -style=file
find ${SRC_DIR} -iname "*.c" | xargs clang-format -i -style=file
find ${INCLUDE_DIR} -iname "*.hpp" | xargs clang-format -i -style=file
find ${TEST_DIR} -iname "*.hpp" | xargs clang-format -i -style=file
