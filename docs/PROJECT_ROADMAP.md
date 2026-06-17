# 项目路线图

## 1. 当前阶段

当前仓库是公开发布准备阶段，重点是：

- 明确移动抓取系统的技术主线。
- 建立 GitHub 仓库结构。
- 准备 CSDN 单篇文章。
- 为后续源码、配置、脚本和图片预留目录。

## 2. 下一阶段：补工程源码

建议按下面顺序补源码：

1. `src/navigation`：底盘、雷达、SLAM、Nav2 bringup。
2. `src/perception`：相机驱动适配、目标检测、深度反投影、手眼坐标转换。
3. `src/manipulation`：机械臂驱动、夹爪控制、MoveIt 配置、IK 示例。
4. `src/task_manager`：自主抓取任务状态机。

## 3. 第三阶段：补配置和示例

建议补：

- `config/nav2_params.yaml`
- `config/slam_params.lua` 或 `config/slam_params.yaml`
- `config/camera_topics.yaml`
- `config/handeye_transform.yaml`
- `config/grasp_profiles.yaml`
- `launch/mobile_manipulation_full.launch.py`

## 4. 第四阶段：补图片和演示

建议补：

- 系统实物图
- RViz TF 树图
- 建图截图
- 导航截图
- 目标检测截图
- 深度图截图
- 机械臂抓取过程图

## 5. 第五阶段：发布

建议发布顺序：

1. GitHub 新建仓库。
2. 上传当前结构和主文。
3. 在 README 中确认源码状态。
4. CSDN 发布 `docs/csdn/csdn_article.md`。
5. CSDN 中加入 GitHub 仓库链接。
6. 后续更新优先提交 GitHub。
