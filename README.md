# MultiSensor_fusion

**说明：使用视觉-惯性和纯激光里程计，通过因子图后处理全局位姿。其中平移的权重激光比例高，旋转的权重视觉-惯性比例高**

**其中视觉前端使用mask-rcnn剔除动态特征点,激光前端采用lio_livox中的前景检测算法实现对动态点云的剔除，保证构建地图的精度**

**传感器类型：大恒工业相机、Xsens惯性传感器、VLP-32激光雷达、松灵机器人底盘**

##Compilation

cd ~/catkin_ws/src

git clone https://github.com/GuoFeng-X/MultiSensor_fusion.git

cd ..

catkin_make

## Usage
### 1. 视觉前端运行
下载基于ROS话题发布的mask-rcnn前端代码。[mask-rcnn ros版本](https://download.csdn.net/download/qq_37568167/36765493)

拷贝上述代码到MultiSensor_fusion文件夹下

cd catkin_ws/src/MultiSensor_fusion

unzip Mask-RCNN

cd script/mask_rcnn

./run_build.sh

cd ..

./run_detect.sh

## 2. 视觉惯性里程计和激光里程计
source devel/setup.bash

roslaunch MultiSensor_fusion run_fusion.launch

## 3. 运行自己的数据集
rosbag play xxx.bag --pause -r0.5


## Acknowledgements
Thanks for following work:

LOAM (LOAM: Lidar Odometry and Mapping in Real-time)

VINS-Mono (VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator)
