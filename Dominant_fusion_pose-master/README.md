# Dominant_fusion_pose (位姿融合模块)

## 安装bfl(Beyesian filter library)

http://www.orocos.org/bfl
https://blog.csdn.net/zhxue_11/article/details/83828877

### Installation on Ubuntu from precompiled package

- First add the following line to your /etc/apt/sources.list:
```bash
  deb http://people.mech.kuleuven.be/~tdelaet/bfl mydistro main
```
where <b>mydistro</b> is either breezy, dapper, edgy or feisty  not avaliable for Ubuntu 16.04

- Then apt-get the packages:
```bash
  sudo apt-get update
  sudo apt-get install orocos-bfl
```

### Installation from source

https://github.com/toeklk/orocos-bayesian-filtering/wiki/Installation

```bash
git clone https://github.com/toeklk/orocos-bayesian-filtering
cd orocos-bayesian-filtering/orocos_bfl
mkdir build && cd build
make -j8
sudo make install
```

## 离线数据回放

```bash
./SensorSimulator_2019_ubuntu_16_04_x86_64 --flagfile=/dominant/share/sensorsim.flags
```

*注意*修改flags文件中的数据文件夹位置。

## 位姿融合模块启动

```bash
./FusionPose_2019_ubuntu_16.04_x86_64
```

*注意*同一级目录下的FusionPose.yaml是模块的配置文件。
*注意*配置文件中开启的节点需要确认代码中参与了kalman滤波。 (/src/PoseFuse.cpp 227-362 filter_->Update)

## 结果发送

```c++
PacketFusionPos my_data;
...
mlcm.publish("FusionPos", &my_data);
```

## 融合模块算法简介

融合模块采用了EKF，系统采用简化的车辆运动学模型。系统同时估计了车速和车辆朝向角速度。

![SystemModel](https://latex.codecogs.com/gif.latex?X_%7Bt%7D%3D%5Cbegin%7Bbmatrix%7D%20x_%7Bt%7D%5C%5C%20y_%7Bt%7D%5C%5C%20z_%7Bt%7D%5C%5C%20p_%7Bt%7D%5C%5C%20yaw_%7Bt%7D%5C%5C%20r_%7Bt%7D%5C%5C%20v_%7Bt%7D%5C%5C%20w_%7Bt%7D%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%20x_%7Bt-1%7D&plus;sin%28yaw_%7Bt-1%7D%29*v_%7Bt-1%7D*%5CDelta%20t%5C%5C%20y_%7Bt-1%7D&plus;cos%28yaw_%7Bt-1%7D%29*v_%7Bt-1%7D*%5CDelta%20t%5C%5C%20z_%7Bt-1%7D%5C%5C%20p_%7Bt-1%7D%5C%5C%20yaw_%7Bt-1%7D&plus;w_%7Bt-1%7D*%5CDelta%20t%5C%5C%20r_%7Bt-1%7D%5C%5C%20v_%7Bt-1%7D%5C%5C%20w_%7Bt-1%7D%20%5Cend%7Bbmatrix%7D)

$$X_{t}=\begin{bmatrix} x_{t}\\ y_{t}\\ z_{t}\\ p_{t}\\ yaw_{t}\\ r_{t}\\ v_{t}\\ w_{t} \end{bmatrix}=\begin{bmatrix} x_{t-1}*sin(yaw_{t-1})*v_{t-1}*\Delta t\\ y_{t-1}*cos(yaw_{t-1})*v_{t-1}*\Delta t\\ z_{t-1}\\ p_{t-1}\\ yaw_{t-1}*w_{t-1}*\Delta t\\ r_{t-1}\\ v_{t-1}\\ w_{t-1} \end{bmatrix}$$