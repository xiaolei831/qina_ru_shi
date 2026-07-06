# jie_ware ROS2

ROS2 port of the original `jie_ware` lidar localization utilities.

## Nodes

- `lidar_loc`: subscribes to `/map`, `/scan`, `/odom`, and `/initialpose`; publishes `map -> odom_combined` and, by default, bridges `/odom` into `odom_combined -> base_footprint`.
- `lidar_filter_node`: subscribes to `/scan` and publishes `/scan_filtered` after single-point outlier removal.
- `costmap_cleaner`: clears Nav2 global and local costmaps when `/initialpose` is received.

## Build

```bash
cd /home/sunrise/qian_sai
colcon build --packages-select jie_ware
source install/setup.bash
```

## Run Lidar Localization Only

```bash
ros2 launch jie_ware lidar_loc.launch.py
```

For the qian_sai navigation stack, `nav2_qing.launch.py` starts `jie_ware/lidar_loc` directly and does not start AMCL or Cartographer.
