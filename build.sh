#!/bin/bash
mkdir output
cd output
rm -r *
cd ..
mkdir build
cd build
rm -r *
cmake ..
make
./POSITION_ANALYZE
