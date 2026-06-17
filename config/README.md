# config

这个目录放工程参数文件。

| 文件 | 作用 |
| --- | --- |
| `perception_3d.yaml` | 目标检测模拟参数、深度反投影参数 |
| `grasp_planner.yaml` | 机械臂工作空间、预抓取偏移 |
| `task_manager.yaml` | 任务状态机节拍和自动启动 |
| `frames.yaml` | 常用坐标系名称 |

接真实机器人时，优先修改配置文件，不要直接改节点源码。
