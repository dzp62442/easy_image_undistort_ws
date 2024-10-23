# easy_image_undistort_ws

[ethz-asl/image_undistort](https://github.com/ethz-asl/image_undistort) 项目的简化版本，仅保留了相机参数和去畸变模块，主要用于 [kalibr](https://github.com/ethz-asl/kalibr) 标定后的 omni + radtan 相机模型去畸变。

## 代码结构

- `camera_parameters` 和 `undistorter` 头文件和源文件均来自于 [ethz-asl/image_undistort](https://github.com/ethz-asl/image_undistort) 源项目，未改动
- `easy_image_undistort_node.cpp` 为自建 ROS 服务器节点，提供去畸变服务
- `test_undistort_client.py` 为自建 ROS 客户端测试脚本，用于测试去畸变服务

```shell
easy_image_undistort  # ROS 功能包
├── CMakeLists.txt
├── config
│   └── white-car-calib-data.yaml  # kalibr 输出的标定文件
├── include
│   └── easy_image_undistort
│       ├── camera_parameters.h
│       └── undistorter.h
├── launch
│   └── undistort_white_car.launch
├── package.xml
├── scripts
│   └── test_undistort_client.py  # 客户端测试脚本
├── src
│   ├── camera_parameters.cpp
│   ├── easy_image_undistort_node.cpp  # 服务器节点
│   └── undistorter.cpp
└── srv
    └── ImageUndistort.srv
```

## 编译运行

```shell
catkin_make
roslaunch easy_image_undistort undistort_white_car.launch
rosrun easy_image_undistort test_undistort_client.py
```