#!/bin/bash

set -e

grep -E "[\*/#]+\s*TODO" $(find . -name "*.cpp") $(find . -name "*.hpp")

if [ $? -eq 0 ]
then
	exit 1
else
	exit 0
fi
