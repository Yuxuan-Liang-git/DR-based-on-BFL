#!/bin/bash
./build.sh
/bin/python3 /home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/position-analyze-bfl/src/plot.py            //  各子参数对比图
/bin/python3 /home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/position-analyze-bfl/src/plot_pos.py        //  相对位移
# /bin/python3 /home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/position-analyze-bfl/src/plot_gps_pos.py    //  gps的相对位移

/bin/python3 /home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/position-analyze-bfl/src/pos_visualization.py    //  位移在地图上的投影
