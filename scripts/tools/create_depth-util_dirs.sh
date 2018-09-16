#!/bin/bash

mkdir ../build/src/vision/depth-utils/RGBD_DATA

for i in 0 1 2 3 4
do
	mkdir ../build/src/vision/depth-utils/RGBD_DATA/DepthImage$i
	mkdir ../build/src/vision/depth-utils/RGBD_DATA/RGB$i
	mkdir ../build/src/vision/depth-utils/RGBD_DATA/Combined$i
	mkdir ../build/src/vision/depth-utils/RGBD_DATA/Cloud$i
	mkdir ../build/src/vision/depth-utils/RGBD_DATA/RCloud$i
done

