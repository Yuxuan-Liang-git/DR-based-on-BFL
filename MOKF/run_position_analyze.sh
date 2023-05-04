#!/bin/bash
sh build.sh
cd build
./POSITION_ANALYZE
cd ..
/bin/python3 /home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/MOKF/src/MOKF_pos_show.py
