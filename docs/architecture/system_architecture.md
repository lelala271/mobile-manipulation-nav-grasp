# 系统架构说明

## 1. 总体目标

系统目标是实现移动抓取闭环：

```text
底盘自主导航到抓取区域
深度相机识别目标并计算三维坐标
机械臂根据目标位姿完成自主抓取
底盘继续移动到放置区域
机械臂完成放置并回到安全姿态
```

这不是单纯导航，也不是单纯机械臂控制。系统必须同时处理：

- 移动底盘的大范围运动
- 雷达建图、定位和避障
- RGB-D 目标识别与三维定位
- 相机坐标到机械臂坐标的转换
- 机械臂 IK、轨迹规划和夹爪控制
- 任务状态机和失败恢复

## 2. 分层架构

| 层级 | 模块 | 说明 |
| --- | --- | --- |
| 硬件层 | 底盘、雷达、深度相机、机械臂、夹爪、工控机 | 提供运动、感知和计算基础 |
| 驱动层 | 底盘驱动、雷达驱动、相机驱动、机械臂驱动、夹爪驱动 | 把硬件接口转成 ROS 2 数据 |
| 感知层 | SLAM、定位、目标检测、深度定位 | 建立环境地图并识别目标三维位置 |
| 规划层 | Nav2、MoveIt、抓取规划 | 生成底盘路径和机械臂轨迹 |
| 执行层 | `/cmd_vel`、关节轨迹、夹爪命令 | 真正控制底盘和机械臂 |
| 任务层 | 状态机、行为树、任务 Action | 编排完整任务 |

## 3. ROS 2 节点建议

```text
base_driver
lidar_driver
camera_driver
robot_state_publisher
slam_or_localization
nav2_bringup
object_detector
depth_projector
handeye_transformer
grasp_planner
arm_driver
gripper_driver
task_manager
```

## 4. 关键数据流

```text
/scan + /odom + /tf
-> SLAM / localization
-> map 与机器人位姿
-> Nav2
-> /cmd_vel
-> 底盘运动
```

```text
/color/image_raw + /depth/image_raw + /camera_info
-> 目标检测
-> 深度反投影
-> camera 坐标系目标点
-> hand-eye transform
-> arm_base 坐标系目标点
-> grasp planner
-> IK / trajectory
-> arm + gripper
```

## 5. 坐标系要求

必须明确：

- `map`：全局地图坐标系
- `odom`：局部里程计坐标系
- `base_link`：底盘坐标系
- `laser_link`：雷达坐标系
- `camera_link`：相机安装坐标系
- `camera_color_optical_frame`：相机光学坐标系
- `arm_base_link`：机械臂基座坐标系
- `tool0`：机械臂末端工具坐标系
- `gripper_link`：夹爪坐标系

抓取失败时优先检查 TF，因为视觉和机械臂之间所有几何关系都依赖 TF。
