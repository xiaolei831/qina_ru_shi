# 机械臂视觉抓取调参说明

本文档说明 `robot_arm_control` 中“视觉识别目标后，底盘对准目标，再由机械臂执行固定姿态抓取”的流程和参数调整方法。

## 启动流程

启动命令：

```bash
source /home/sunrise/qian_sai/install/setup.bash
ros2 launch robot_arm_control medicine_detect.launch.py
```

启动后主要节点关系：

```text
Astra camera
  -> /camera/color/image_raw
  -> /camera/depth/image_raw
visual_tracker.py
  -> /color_position
  -> /bpu_detection/image
car_location_color.py
  -> 订阅 /color_position，控制底盘对准目标
  -> /arm_state
direct_arm_pick_and_put.py
  -> 订阅 /arm_state，执行 pick / put / rotate_put 等动作
```

当前流程：

```text
视觉识别药品
-> 发布目标在画面中的角度偏差和深度 /color_position
-> 底盘根据 angle_x / distance 对准目标
-> 对准稳定后发布 /arm_state=pick
-> 机械臂执行夹取动作
```

## 关键文件

参数文件：

```text
/home/sunrise/qian_sai/src/robot_arm_control/config/medicine_detect_params.yaml
```

视觉识别节点：

```text
/home/sunrise/qian_sai/src/robot_arm_control/scripts/visual_tracker.py
```

机械臂控制节点：

```text
/home/sunrise/qian_sai/src/robot_arm_control/scripts/direct_arm_pick_and_put.py
```

## 参数说明

以下参数都在 `medicine_detect_params.yaml` 中。

### 是否开启机械臂视觉追踪

```yaml
visual_arm_tracking_enabled: true
```

含义：历史机械臂视觉追踪参数，当前固定姿态直接抓取流程不使用。

可选值：

- `true`：开启机械臂视觉追踪。
- `false`：关闭追踪，只执行固定机械臂动作。

当前流程保持：

```yaml
visual_arm_tracking_enabled: false
```

### 左右追踪增益

```yaml
visual_track_gain_joint_1: 0.25
```

含义：把视觉水平角度偏差 `angle_x` 转换成 `joint_1` 的转动量。

`joint_1` 主要负责机械臂左右摆动。

调整规则：

- 数值越大，左右追踪越快。
- 数值越小，左右追踪越慢、更稳。
- 如果左右方向追反了，把正负号反过来。

示例：

```yaml
visual_track_gain_joint_1: 0.25
```

如果目标在画面右边，但机械臂越追越偏，改成：

```yaml
visual_track_gain_joint_1: -0.25
```

### 上下追踪增益

```yaml
visual_track_gain_joint_2: 0.20
```

含义：把视觉垂直角度偏差 `angle_y` 转换成 `joint_2` 的转动量。

`joint_2` 主要负责机械臂上下抬落。

调整规则：

- 数值越大，上下追踪越快。
- 数值越小，上下追踪越慢、更稳。
- 如果上下方向追反了，把正负号反过来。

示例：

```yaml
visual_track_gain_joint_2: 0.20
```

如果目标在画面上方，但机械臂越追越偏，改成：

```yaml
visual_track_gain_joint_2: -0.20
```

### 单次最大关节步长

```yaml
visual_track_max_joint_step: 0.03
```

含义：机械臂每次根据视觉误差最多转动多少弧度。

当前值 `0.03` 约等于单次最多转动 `1.7 度`。

调整规则：

- 动作太猛、容易抖：调小。
- 动作太慢、追不上：调大。
- 初次调试不建议超过 `0.03`。

稳一点的值：

```yaml
visual_track_max_joint_step: 0.015
```

快一点的值：

```yaml
visual_track_max_joint_step: 0.05
```

### 追踪中心死区

```yaml
visual_track_center_tolerance_x: 0.03
visual_track_center_tolerance_y: 0.03
```

含义：当视觉偏差小于该阈值时，机械臂不再继续微调。

作用：防止目标已经接近中心后，机械臂仍然来回抖动。

调整规则：

- 机械臂一直抖：调大。
- 目标没对准就停止：调小。

更稳：

```yaml
visual_track_center_tolerance_x: 0.05
visual_track_center_tolerance_y: 0.05
```

更准：

```yaml
visual_track_center_tolerance_x: 0.02
visual_track_center_tolerance_y: 0.02
```

### 关节限位

```yaml
visual_track_joint_1_min: -1.2
visual_track_joint_1_max: 1.2
visual_track_joint_2_min: -0.2
visual_track_joint_2_max: 1.2
```

含义：限制视觉追踪时 `joint_1` 和 `joint_2` 的可运动范围，避免机械臂追踪过大。

单位：弧度。

当前含义：

- `joint_1` 限制在 `-1.2 ~ 1.2 rad`。
- `joint_2` 限制在 `-0.2 ~ 1.2 rad`。

调整规则：

- 左右摆动范围不够：适当放宽 `joint_1` 限位。
- 上下抬落范围不够：适当放宽 `joint_2` 限位。
- 初次调试建议保持较保守范围。

建议安全值：

```yaml
visual_track_joint_1_min: -1.2
visual_track_joint_1_max: 1.2
visual_track_joint_2_min: -0.2
visual_track_joint_2_max: 1.2
```

### 夹取触发中心阈值

```yaml
trigger_center_tolerance_x: 0.04
trigger_center_tolerance_y: 0.04
```

含义：视觉节点认为目标“已经居中，可以夹取”的偏差阈值。

注意：这个参数控制的是何时发布 `/arm_state=pick`，不是机械臂追踪速度。

调整规则：

- 夹取得太早，目标还没对准：调小。
- 一直不夹，实际已经差不多对准：调大。

更严格：

```yaml
trigger_center_tolerance_x: 0.03
trigger_center_tolerance_y: 0.03
```

更容易触发：

```yaml
trigger_center_tolerance_x: 0.06
trigger_center_tolerance_y: 0.06
```

### 夹取触发连续帧数

```yaml
trigger_required_count: 5
```

含义：目标连续多少帧保持居中后，才发布 `/arm_state=pick`。

调整规则：

- 误触发、夹太早：调大。
- 目标对准后迟迟不夹：调小。

更稳：

```yaml
trigger_required_count: 8
```

更快：

```yaml
trigger_required_count: 3
```

## 推荐调参顺序

### 第一步：确认追踪方向

启动后观察日志：

```text
视觉追踪运动: angle_x=... angle_y=... step_y=... step_z=...
```

如果目标越追越偏，先不要调速度，先改方向。

左右反了：

```yaml
visual_track_gain_joint_1: -0.25
```

上下反了：

```yaml
visual_track_gain_joint_2: -0.20
```

一次只改一个方向，方便判断。

### 第二步：调追踪速度

如果动作太猛、抖动明显：

```yaml
visual_track_gain_joint_1: 0.15
visual_track_gain_joint_2: 0.12
visual_track_max_joint_step: 0.015
```

如果动作太慢：

```yaml
visual_track_gain_joint_1: 0.35
visual_track_gain_joint_2: 0.30
visual_track_max_joint_step: 0.05
```

### 第三步：调停止精度

如果目标接近中心后机械臂还一直小幅抖动：

```yaml
visual_track_center_tolerance_x: 0.05
visual_track_center_tolerance_y: 0.05
```

如果目标还没对准就停止：

```yaml
visual_track_center_tolerance_x: 0.02
visual_track_center_tolerance_y: 0.02
```

### 第四步：调夹取触发

如果夹取太早：

```yaml
trigger_center_tolerance_x: 0.03
trigger_center_tolerance_y: 0.03
trigger_required_count: 8
```

如果目标已经对准但一直不夹：

```yaml
trigger_center_tolerance_x: 0.06
trigger_center_tolerance_y: 0.06
trigger_required_count: 3
```

## 推荐初始配置

建议先用稳一点的配置：

```yaml
visual_arm_tracking_enabled: true
visual_track_gain_joint_1: 0.20
visual_track_gain_joint_2: 0.15
visual_track_max_joint_step: 0.02
visual_track_center_tolerance_x: 0.04
visual_track_center_tolerance_y: 0.04
visual_track_joint_1_min: -1.2
visual_track_joint_1_max: 1.2
visual_track_joint_2_min: -0.2
visual_track_joint_2_max: 1.2

trigger_center_tolerance_x: 0.05
trigger_center_tolerance_y: 0.05
trigger_required_count: 5
```

如果确认机械臂不会撞桌子、动作方向也正确，再逐步提高速度。

## 常见现象和处理

### 目标越追越偏

原因：方向反了。

处理：

```yaml
visual_track_gain_joint_1: -0.25
```

或：

```yaml
visual_track_gain_joint_2: -0.20
```

### 机械臂动作太猛

原因：增益或单次步长太大。

处理：

```yaml
visual_track_gain_joint_1: 0.15
visual_track_gain_joint_2: 0.12
visual_track_max_joint_step: 0.015
```

### 机械臂一直抖动

原因：目标检测有轻微跳动，死区太小。

处理：

```yaml
visual_track_center_tolerance_x: 0.05
visual_track_center_tolerance_y: 0.05
```

### 对准后不夹取

原因：夹取触发阈值太严格，或连续帧数太多。

处理：

```yaml
trigger_center_tolerance_x: 0.06
trigger_center_tolerance_y: 0.06
trigger_required_count: 3
```

### 夹取太早

原因：夹取触发阈值太宽，或连续帧数太少。

处理：

```yaml
trigger_center_tolerance_x: 0.03
trigger_center_tolerance_y: 0.03
trigger_required_count: 8
```

### 机械臂运动范围太大

原因：关节限位太宽或步长太大。

处理：

```yaml
visual_track_max_joint_step: 0.015
visual_track_joint_2_min: 0.0
```

## 注意事项

- 每次改完参数后，需要重新启动 launch，参数才会重新加载。
- 初次调试时建议把速度调慢，确认方向正确后再提高速度。
- 当前视觉追踪使用关节空间微调，主要调 `joint_1` 和 `joint_2`，比笛卡尔 IK 更容易让机械臂从观察位稳定运动。
- 当前视觉追踪没有根据深度计算目标三维坐标。
- 当前流程不启动 `car_location_color_node`，因此视觉追踪不会控制底盘 `/cmd_vel`。
