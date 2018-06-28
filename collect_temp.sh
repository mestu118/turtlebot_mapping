#!/bin/bash

if [ "$#" -ne "2" ]; then
  echo "Please pass target directory and prefix"
  exit 1
fi

mkdir -p $1
target_dir=`realpath $1`
prefix=$2

echo "Target directory: $target_dir"
echo "Rosbag prefix:    $prefix"
echo "$target_dir/${prefix}.bag"

rosbag record -O $target_dir/${prefix}.bag \
	/camera/depth_registered/image_raw \
	/camera/rgb/camera_info \
	/camera/rgb/image_color \
	/camera/rgb/image_raw \
	/odom \
	/tf 
	


