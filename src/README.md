# src

这个目录包含 ROS 2 工程源码包。

| 包 | 作用 |
| --- | --- |
| `perception_3d` | 目标检测模拟节点、深度反投影节点 |
| `grasp_planner` | 把目标三维点转换成预抓取和抓取位姿 |
| `task_manager` | 移动抓取任务状态机 |
| `mobile_manipulation_bringup` | 配置和启动文件 |

## 编译方式

```bash
cd ~/mobile_manipulation_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

这些包是工程起点，不绑定具体硬件。接真实设备时，需要把模拟检测、固定目标点和默认相机内参替换成真实驱动与标定结果。
