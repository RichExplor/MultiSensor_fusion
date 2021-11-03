# 需要注意，需要重新编译才能使用
## 1. 打开虚拟环境
source activate pytorch

## 2. 编译
cd detectron2
python setup.py build develop

## 3. 运行
cd detectron2/Visual_Frand

./run_detect.sh

### 3.1 重启终端
roslaunch vins_estimator superpoint_mynteye.launch

roslaunch vins_estimator vins_rviz.launch

### 3.2 打开数据集
rosbag play -r0.5 mynteye_02.bag
