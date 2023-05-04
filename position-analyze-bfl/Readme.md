# 使用方式

`sh ./run_pos_f.sh`

# 文件说明

* run_pos_f.sh：整个项目的总脚本

* build.sh：编译脚本

* src

  * main.cpp：主函数 调用数据处理等
  * Raw_data.cpp：从data/OfflineData.txt提取数据
  * Filtered_data.cpp：继承Raw_data，调用bfl库进行滤波与数据融合
  * nonlinearanalyticconditionalgaussianmobile.cpp：定义EKF模型相关参数（如雅可比矩阵等）
  * plot.py：绘制各数据经滤波后的对比图
  * plot_pos.py：绘制滤波前、滤波后以及GPS坐标对应的相对轨迹图（以起始点为零点）
  * plot_gps_pos.py：仅绘制GPS坐标轨迹图
  * plot_visualization.py：将相对轨迹图对应的实际经纬度绘制在地图中

* include

  * Raw_data.h
  * Filtered_data.h
  * nonlinearanalyticconditionalgaussianmobile.h

* doc：可能用到的文档

* output：输出的图片全存在这里

  
