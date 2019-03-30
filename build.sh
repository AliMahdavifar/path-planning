#!/bin/bash

mkdir build && cd build
cmake ..
make
cp pathplanner ..
cd ..
rm -R -f build

clear
echo "Execute ./pathplanner Config.yaml"
sleep 1

./pathplanner Config.yaml