#!/bin/bash

# This checks if the required directories exist and if not
# then they are created.

if ! test -d "../nav"
then
	echo "Please run from nav root."
	exit 1
fi

if test -d "DataCoordImages";
then
	echo "DataCoordImages is being cleared"
	rm -rf DataCoordImages
	mkdir DataCoordImages
else
	mkdir DataCoordImages
fi

if test -d "DataCoordImages/0";
then
	echo "DataCoordImages/0 already exists."
else
	mkdir DataCoordImages/0
fi

if test -d "DataCoordImages/1";
then
	echo "DataCoordImages/1 already exists."
else
	mkdir DataCoordImages/1
fi

if test -d "DataCoordImages/2";
then
	echo "DataCoordImages/2 already exists."
else
	mkdir DataCoordImages/2
fi

if test -d "DataCoordImages/3";
then
	echo "DataCoordImages/3 already exists."
else
	mkdir DataCoordImages/3
fi

if test -d "DataCoordImages/4";
then
	echo "DataCoordImages/4 already exists."
else
	mkdir DataCoordImages/4
fi

