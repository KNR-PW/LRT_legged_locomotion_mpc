# LRT_terrain_model
Package that includes terrain model interface and implementations for legged robots

## Dependencies
- [ocs_ros2](https://github.com/BartlomiejK2/ocs2_ros2):
  - ocs2_core
- [grid_map](https://github.com/BartlomiejK2/grid_map)
- [plane_segmentation_ros2](https://github.com/BartlomiejK2/plane_segmentation_ros2)

## Installation 
1. Clone repo to your workspace (HTTPS or SSH)
2. Install dependencies in workspace:
```bash
rosdep install --ignore-src --from-paths . -y -r
```
3. Build:
```bash
colcon build --packages-select terrain_model (not working yet xD)
```
