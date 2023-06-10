# Mobile Robotics Assignment GoTo

## Installation
Place the contents of this repo inside $MR_DIR/ws02/src/mr_goto  
`git clone git@github.com:VincentKen/MR-GoTo.git $MR_DIR/ws02/src/mr_goto`

replace ekf_node.cpp and ekf_node.hpp with the ones in MR_EKF
also in your ekf.cpp add `reset_ = Reset::INTI_POSE;` to set_init_pose
`void EKF::set_init_pose(const Pose2D &p)
{
    if (!init_pose_)
        init_pose_ = make_shared<Pose2D>(p);
    else
        *init_pose_ = p;
    reset_ = Reset::INTI_POSE;
}`
## Build
Build using `colcon build`

## Run
Run using `ros2 run mr_goto mr_goto`

### Run with map
We can also choose the map file we want `mr_goto` to run with.
```bash
ros2 run mr_goto mr_goto --ros-args -p map_file:=<path_to_map>
```
For example:
```bash
ros2 launch mr_goto launch.py map:=ws02/src/mr_goto/config/world/bitmaps/cave.png
```
Note: Per default, we are using the `line` map.

## Launch
You first need to execute
```bash
make build-ws02
```
in the project's root directory.

Launch using
```bash
ros2 launch mr_goto launch.py
```

### Launch with map
You can also specify the map you want to use by using an additional `map` argument.
```bash
ros2 launch mr_goto launch.py map:=<map_name>
```
For example:
```bash
ros2 launch mr_goto launch.py map:=cave
```
Note: Per default, we are using the `line` map.

### Optional ekf or pf
For launching the `ekf` or `pf` node, execute
```bash
ros2 launch mr_goto launch.py localization:=<node>
```
For example:
```bash
ros2 launch mr_goto launch.py localization:=ekf
```
