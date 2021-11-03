# MultiSensor_fusion

**说明：使用视觉-惯性和纯激光里程计，通过因子图后处理全局位姿。其中平移的权重激光比例高，旋转的权重视觉-惯性比例高**

**其中视觉前端使用mask-rcnn剔除动态特征点**

**传感器类型：大恒工业相机、Xsens惯性传感器、VLP-32激光雷达、松灵机器人底盘**

## Usage
### 1. 视觉前端运行
cd MultiSensor_fusion/script/mask_rcnn
./run_build.sh

## 2. 视觉惯性里程计和激光里程计
source devel/setup.bash
roslaunch MultiSensor_fusion run_fusion.launch

## 3. 运行自己的数据集
rosbag play xxx.bag --pause -r0.5
