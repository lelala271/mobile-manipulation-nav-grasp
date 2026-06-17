# 发布计划

## 主题修正

本项目公开发布主题应定位为：

```text
移动抓取机器人完整工程：雷达导航避障 + 深度相机三维识别 + 机械臂自主抓取
```

不要把主题写成单独的 Cartographer 教程。Cartographer 可以作为雷达建图章节出现，但不是整篇文章的主线。

## GitHub 负责什么

GitHub 适合承载完整工程：

- 主 README 长文
- 源码目录
- launch 文件
- 参数配置
- 标定说明
- 模型权重说明
- 部署脚本
- 故障排查文档

## CSDN 负责什么

CSDN 适合承载单篇技术文章：

- 讲清楚系统整体路线
- 给读者建立模块分类
- 给出关键流程、表格和排错方法
- 引导读者去 GitHub 获取源码、配置和后续更新

## 建议发布顺序

1. 先在 GitHub 建仓库并上传当前结构。
2. GitHub README 使用仓库根目录的 `README.md`。
3. CSDN 使用 `docs/csdn/csdn_article.md`。
4. CSDN 文章开头或结尾加入 GitHub 仓库链接。
5. 后续源码、配置和标定文件优先更新 GitHub，CSDN 只维护入口和核心讲解。

## 仓库标题建议

```text
mobile-manipulation-nav-grasp
```

## CSDN 标题建议

```text
移动抓取机器人从 0 到 1：雷达导航、深度相机识别、机械臂自主抓取一次讲清
```

## GitHub 仓库简介建议

```text
ROS 2 mobile manipulation engineering guide: lidar navigation, RGB-D 3D perception, hand-eye calibration, inverse kinematics, manipulation planning, and autonomous grasp task flow.
```
