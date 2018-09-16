#!/bin/sh

cppcheck --enable=warning,performance,portability,unusedFunction \
	--error-exitcode=1 \
	--quiet \
	--std=c++11 \
	-i src/msg \
	-I src/nav \
	-I src/gcs \
	-I src/sim \
	.
