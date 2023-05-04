#! https://zhuanlan.zhihu.com/p/624857096
# 调用BFL库实现数据融合与拓展卡尔曼滤波
## 梁宇翾/阿环 2023.04.25
![Image](https://pic4.zhimg.com/80/v2-05cc4d141eb7b1801702935ac2b3ea85.jpg)
## 介绍
该库集成了卡尔曼滤波、拓展卡尔曼滤波、粒子滤波等功能。在此不多做赘述，会用就行。基本介绍可以看
>https://www.bilibili.com/read/cv15868405FL库教程基本介绍

安装和基本的使用可以看英文文档
>https://gitee.com/ribbite/orocos-bfl/blob/master/docs/getting_started_guide.pdf

**写这篇教程的目的是记录下研究了一两周才搞明白例程里没有讲到的多传感器融合相关的使用方法。**
## 基本使用介绍
照着教程安装好BFL库
>https://www.bilibili.com/read/cv15827495?spm_id_from=333.999.0.0

我用了CMAKE，在CMakeLists.txt里添加bfl库的头文件地址、orocos预编译好的.so文件以及运算需要的矩阵库EIGEN（SOPHUS不知道要不要，反正我别的地方用到了）
``` cpp
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
INCLUDE_DIRECTORIES(  
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    /usr/local/include/bfl)

link_directories(
  /usr/local/lib
)
```
直接从例程“nonlinear_kalman”入手。头文件`nonlinearanalyticconditionalgaussianmobile.h`是自己写的，其对应的`nonlinearanalyticconditionalgaussianmobile.cpp`文件也是自己定义的。
拓展卡尔曼滤波针对的是非线性系统，需要将系统取偏导后近似线性化。其雅可比矩阵在这个文件里自己定义。具体操作可以看
>https://www.bilibili.com/read/cv16037356

按以下顺序初始化系统即可：

```cpp
//  定义系统不确定性参数 传入系统噪声
Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);
//  根据不确定性参数确定系统的概率分布密度函数
//  NonLinearAnalyticConditionalGaussianMobile这个类自己写的，雅可比矩阵也在里头
NonLinearAnalyticConditionalGaussianMobile sys_pdf(system_Uncertainty);
// 创建系统模型
AnalyticSystemModelGaussianUncertainty sys_model(&sys_pdf);

//  和创建系统模型一样的操作，创建带有噪声的观测模型
Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);
//  创建观测模型 注意H是观测量到状态量的转移矩阵
LinearAnalyticConditionalGaussian meas_pdf(H, measurement_Uncertainty);
LinearAnalyticMeasurementModelGaussianUncertainty meas_model(&meas_pdf);

//  设置先验概率 创建EKF模型
Gaussian prior_cont(prior_Mu,prior_Cov);
ExtendedKalmanFilter filter(&prior_cont);
```
在主循环中调用`filter.Update(&sys_model,input,&meas_model,measurement)`即可更新系统。
想获得状态量（预测量与观测量卡尔曼滤波后得到的更可信的数据），👇
```cpp
ColumnVector state(7);
state = filter.PostGet()->ExpectedValueGet();
//  ExpectedValueGet是在nonlinearanalyticconditionalgaussianmobile.h里头自己写的，不同状态量之间关系的函数
```
## 多观测量的使用
其实也很简单，系统的更新和观测量的更新可以分开，在主循环里我们更新系统`filter.Update(&sys_model,input)`，当我们得到观测值的时候更新观测模型`filter.Update(&meas_model,measurement)`即可。我们可以搞很多个观测模型观测系统状态量，记得更新观测模型即可。

数据融合后的航向轨迹可以做到下面的效果（纯底盘数据 无IMU 仅低频GPS信号修正航位）
轨迹图是将经纬度用plotly画出来的
![Image](https://pic4.zhimg.com/80/v2-a15c30e32009c215aba4061e07129853.png)