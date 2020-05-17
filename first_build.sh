#!/bin/bash

cd thirdparty/g2o
mkdir build && cd build
cmake .. && make -j2

echo g2o is built !

cd ../..

mkdir build && cd build
cmake .. && make -j2

echo Done!
