#!/bin/bash

if (( $# != 1 )); then
  echo "USAGE: make_bin_directory.sh <output_directory>"
  exit 1  
fi
echo "This will create a new binary release at the directory:"
echo "    $1"
echo "--> If the directory exists it will be deleted."
read -p "--> Continue (y|n)? " -n 1 -r choice
echo    # (optional) move to a new line
if [[ $choice == y ]]
then
  echo "creating binary version..."
else
  echo "exiting..."
  exit 1
fi

if [ -d "$1" ]; then
  echo "Deleting directory: $1"
  rm -rf "$1"
fi

echo "Creating directory: $1"
mkdir $1

echo "Copying files..."
cp ./KinectHands/build/x64/Release/KinectHands.exe $1/KinectHands.exe
mkdir "$1/data"
if [ ! -f ./data/forest_data.bin ] 
then
  unzip -o ./data/forest_data.zip -d ./data
fi
cp ./data/forest_data.bin $1/data/forest_data.bin
cp -r ./resource_files $1/resource_files
cp -r ./models $1/models
cp -r ./shaders $1/shaders
cp settings.csv $1/settings.csv
cp depth_undistort_lookup_table.bin $1/depth_undistort_lookup_table.bin