# robot_arm_control - BPU Medicine Detection And Direct Grasping

## Features

This ROS 2 package uses an RGB-D camera and BPU model to detect medicine targets, aligns the chassis with the target, and triggers a direct joint-trajectory grasp sequence.

## Runtime Flow

```text
RGB-D Camera -> visual_tracker.py -> /color_position
                         |
                         v
                 car_location_color.py -> /cmd_vel
                         |
                         v
                    /arm_state=pick
                         |
                         v
              direct_arm_pick_and_put.py
                         |
                         v
      /communication_base/*/joint_trajectory
```

## Nodes

| Node | Language | Purpose |
|------|----------|---------|
| `visual_tracker.py` | Python | Runs BPU medicine detection and publishes `/color_position`. |
| `car_location_color.py` | Python | Aligns the chassis using `/color_position` and publishes `/arm_state=pick`. |
| `direct_arm_pick_and_put.py` | Python | Executes fixed-pose pick and put actions by publishing joint trajectories. |
| `serial_bridge.py` | Python | Optional serial bridge for a separate arm controller. |

## Launch

```bash
ros2 launch robot_arm_control medicine_detect.launch.py
```

The legacy launch name is still available and forwards to the medicine detection launch:

```bash
ros2 launch robot_arm_control arm_pick_color.launch.py
```

## Main Files

- `launch/medicine_detect.launch.py`
- `launch/arm_pick_color.launch.py`
- `scripts/visual_tracker.py`
- `scripts/car_location_color.py`
- `scripts/direct_arm_pick_and_put.py`
- `scripts/serial_bridge.py`
- `config/medicine_detect_params.yaml`
- `model/data2_best_bayese_640x640_nv12.bin`

## Notes

- The removed HSV color-block sorting flow is no longer part of this package.
- Camera input defaults to Astra topics `/camera/color/image_raw` and `/camera/depth/image_raw`.
- The BPU detector depends on the external `hrt_model_exec` command being available at runtime.
