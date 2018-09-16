#!/bin/bash

if [ -d "../build/src/vision/depth-utils/ImageData" ]
then
	j=0
	while [ -e "../build/src/vision/depth-utils/ImageData$j.tar.gz" ]
	do
		((j++))
	done

	tar -czf ../build/src/vision/depth-utils/ImageData$j.tar.gz ../build/src/vision/depth-utils/ImageData/*
	rm -rf ../build/src/vision/depth-utils/ImageData
fi

mkdir ../build/src/vision/depth-utils/ImageData
mkdir ../build/src/vision/depth-utils/ImageData/Timestamps

for i in {0..5}
do
	mkdir ../build/src/vision/depth-utils/ImageData/RGB$i
	mkdir ../build/src/vision/depth-utils/ImageData/Depth$i
done

