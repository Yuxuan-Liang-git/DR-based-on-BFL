# 文件说明

* Dominant_fusion_pose-master：前任做的bfl库在多线程实时接受数据的应用。bfl库调用的有问题（雅可比阵写错了），不能参考。
* matlab：`Multi-sensor optimal information fusion Kalman filter`论文算法在matlab中的复现
* MOKF：以上论文算法的应用 后面发现比较复杂，就放弃了，改用BFL库
* position-analyze-bfl：调用bfl库进行EKF滤波


# 后续工作方向

* 接入IMU数据进行融合
* 换成多线程实时接受数据处理
