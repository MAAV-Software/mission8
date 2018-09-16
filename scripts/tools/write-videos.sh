#!/bin/bash

# Will write videos from image files
# Requires the fps to be passed in as a command line argument

# Verifies that this is the nav root
# if not will notify and exit script

if ! test -d "../nav";
then
	echo "Please run from nav root."
	exit 1;
fi

# Checks for command line arg and sets fps

if [ -z "$1" ];
then
	echo "Default fps is 30.0, other can be passed as command line argument."
	let F_P_S=30
else
	printf "Using inputted fps of " "$1"
	let F_P_S=$1
fi

# Renames all OutTimestamps files by adding the correct postfix

let FILE_INDEX=0
while test -e "OutTimestamps${FILE_INDEX}.txt";
do
	let NEW_PREFIX=0
	while test -e "OutTimestamps${FILE_INDEX}_${NEW_PREFIX}.txt";
	do
		let NEW_PREFIX+=1
	done
	mv "OutTimestamps${FILE_INDEX}.txt" "OutTimestamps${FILE_INDEX}_${NEW_PREFIX}.txt"
	let FILE_INDEX+=1
done

# Find a suitable new postfix for new video files

let FILE_INDEX=0
while test -e "Output_File_0_$FILE_INDEX.avi";
do
	let FILE_INDEX+=1
done

# Move and re-create the DataCoordImages Directory so that all images persist
let FILE_INDEX=0
while test -e "OutDataCoordImages_${FILE_INDEX}";
do
	let FILE_INDEX+=1
done

mv DataCoordImages* OutDataCoordImages_${FILE_INDEX}

# Recreate the DataCoordImages directory

./bash/setup-data-coord-dirs.sh

# Write the new videos with files from DataCoordinator

ffmpeg -r $F_P_S -f image2 -s 640x480 -i OutDataCoordImages_${FILE_INDEX}/0/%d.png -vcodec libx264 -crf 20 -pix_fmt yuv420p "Output_File_0_$FILE_INDEX.avi"
ffmpeg -r $F_P_S -f image2 -s 640x480 -i OutDataCoordImages_${FILE_INDEX}/1/%d.png -vcodec libx264 -crf 20 -pix_fmt yuv420p "Output_File_1_$FILE_INDEX.avi"
ffmpeg -r $F_P_S -f image2 -s 640x480 -i OutDataCoordImages_${FILE_INDEX}/2/%d.png -vcodec libx264 -crf 20 -pix_fmt yuv420p "Output_File_2_$FILE_INDEX.avi"
ffmpeg -r $F_P_S -f image2 -s 640x480 -i OutDataCoordImages_${FILE_INDEX}/3/%d.png -vcodec libx264 -crf 20 -pix_fmt yuv420p "Output_File_3_$FILE_INDEX.avi"
ffmpeg -r $F_P_S -f image2 -s 640x480 -i OutDataCoordImages_${FILE_INDEX}/4/%d.png -vcodec libx264 -crf 20 -pix_fmt yuv420p "Output_File_4_$FILE_INDEX.avi"

# Exit the script with code 0 (no errors)

exit 0;
