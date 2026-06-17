# 主题整改审计

## 1. 原偏离点

前一版构思把重点过度放在 Cartographer 单算法讲解上。这个方向适合写 SLAM 教程，但不适合作为本项目的公开主线。

本项目真正主题应该是：

```text
移动抓取机器人完整工程
```

其中 Cartographer 只是雷达建图模块的一个可选实现，不应该成为整篇文章的中心。

## 2. 整改后的主线

现在主线调整为：

```text
雷达导航避障
-> 深度相机目标识别和三维定位
-> 手眼标定
-> 机械臂逆运动学和轨迹规划
-> 夹爪抓取
-> 任务状态机闭环
```

## 3. 文件整改结果

| 文件 | 作用 |
| --- | --- |
| `README.md` | GitHub 主文，讲完整系统工程 |
| `docs/csdn/csdn_article.md` | CSDN 单篇发布稿 |
| `docs/architecture/system_architecture.md` | 系统架构和数据流 |
| `docs/deployment/deployment_guide.md` | Linux 与 ROS 2 部署 |
| `docs/troubleshooting/troubleshooting.md` | 常见故障排查 |
| `docs/PUBLICATION_PLAN.md` | GitHub 和 CSDN 发布分工 |
| `docs/PROJECT_ROADMAP.md` | 后续源码和工程资料补齐路线 |

## 4. 审计结论

当前发布包已经从“单算法文章”调整为“完整移动抓取工程发布包”。后续应该围绕源码、配置、标定、launch、演示图继续完善，不再把单个 SLAM 算法作为主标题。
