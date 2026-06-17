# launch

顶层 `launch` 目录只保留说明。真正可安装的 ROS 2 launch 文件在：

```text
src/mobile_manipulation_bringup/launch/demo_pipeline.launch.py
```

编译后运行：

```bash
ros2 launch mobile_manipulation_bringup demo_pipeline.launch.py
```

实际工程中建议继续拆分：

- `base_bringup.launch.py`
- `navigation_bringup.launch.py`
- `camera_bringup.launch.py`
- `arm_bringup.launch.py`
- `grasp_pipeline.launch.py`
- `mobile_manipulation_full.launch.py`
