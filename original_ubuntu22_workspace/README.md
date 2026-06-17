# 原始 Ubuntu ROS 2 工程源码说明

该目录用于存放从 Ubuntu ROS 2 工作环境中抽取出来的真实工程源码。目录名、包名、launch 名、URDF 引用、参数文件路径和插件名尽量保持原样，目的是保证源码在 ROS 2 中仍可按原工程关系编译和定位依赖。

## 目录结构

```text
original_ubuntu22_workspace/
├── ros2_ws/
│   └── src/                  # 原始 ROS 2 工作空间源码
└── camera_deps/              # 相机相关底层依赖源码
```

`ros2_ws/src` 中包含底盘启动、消息定义、雷达驱动、SLAM、Nav2、URDF/mesh、RViz 配置、相机驱动、键盘/手柄控制、跟随示例、Web 视频服务、Qt 控制示例等工程包。

`camera_deps` 中包含相机依赖源码，例如 `libuvc`、`glog`、`magic_enum`。这些目录用于保留原始环境中的依赖构建依据，便于离线环境排查相机 SDK 或底层库问题。

## 已排除内容

为了避免把主机状态和敏感信息发布到开源仓库，以下内容没有纳入该目录：

```text
虚拟机磁盘镜像
系统缓存目录
SSH/GPG 凭据
用户 shell 历史
ROS 运行日志
colcon build/install/log 产物
CMake 中间产物
Python __pycache__
重复压缩包和安装包
```

这些内容不是工程源码。部署时应在目标 Ubuntu 主机上重新执行依赖安装和编译。

## 编译方式

推荐在 Ubuntu 22.04 + ROS 2 Humble 环境中单独创建工作空间：

```bash
mkdir -p ~/robot_ros2_ws/src
cd ~/robot_ros2_ws/src
git clone https://github.com/lelala271/mobile-manipulation-nav-grasp.git repo
cp -a repo/original_ubuntu22_workspace/ros2_ws/src/* .
cd ~/robot_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

如果只调试相机底层依赖，可进入 `repo/original_ubuntu22_workspace/camera_deps` 对应目录按各自 `CMakeLists.txt` 构建。

## 与补充示例工程的关系

根目录 `src/perception_3d`、`src/grasp_planner`、`src/task_manager`、`src/mobile_manipulation_bringup` 是后续补充的移动抓取闭环示例，用于表达“深度相机三维定位 -> 抓取位姿规划 -> 任务状态机”的软件接口关系。

该目录下的 `ros2_ws/src` 是原始环境工程源码，用于保留已有底盘、雷达、导航、相机和机器人模型工程资产。
