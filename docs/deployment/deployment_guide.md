# 部署指南

## 1. 推荐环境

推荐：

```text
Ubuntu 22.04
ROS 2 Humble
Nav2
MoveIt 2
Cartographer 或 SLAM Toolbox
```

如果使用 Ubuntu 20.04，需要重新评估 ROS 版本和依赖兼容性。不要直接复制其他系统的 `build`、`install` 目录作为部署方案。

## 2. 安装 ROS 2 Humble

参考官方文档：

[ROS 2 Humble Ubuntu Install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

```bash
sudo rosdep init
rosdep update
```

## 3. 安装常用组件

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-tf2-tools \
  ros-humble-rviz2
```

MoveIt 2 安装以官方文档为准：

[MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html)

## 4. 创建工作空间

```bash
mkdir -p ~/mobile_manipulation_ws/src
cd ~/mobile_manipulation_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 5. 分阶段启动

第一阶段：底盘和传感器。

```bash
ros2 launch <base_package> base_bringup.launch.py
ros2 launch <lidar_package> lidar.launch.py
ros2 launch <camera_package> camera.launch.py
```

第二阶段：建图或定位。

```bash
ros2 launch <navigation_package> slam.launch.py
```

第三阶段：导航。

```bash
ros2 launch <navigation_package> nav2.launch.py map:=/path/to/map.yaml
```

第四阶段：机械臂。

```bash
ros2 launch <arm_package> arm_bringup.launch.py
ros2 launch <moveit_config_package> move_group.launch.py
```

第五阶段：抓取闭环。

```bash
ros2 launch <grasp_package> grasp_pipeline.launch.py
ros2 launch <task_package> task_manager.launch.py
```

## 6. 验收命令

```bash
ros2 topic list
ros2 node list
ros2 topic hz /scan
ros2 topic echo /odom
ros2 run tf2_tools view_frames
ros2 action list
```

部署完成不等于系统可用。必须逐层验证：底盘、雷达、相机、机械臂、夹爪、导航、识别、抓取状态机。
