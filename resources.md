# Resources
## Commands
### Running Cartographer
Always Run this on PI first:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Then run this on local pc:
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

If you want to use a premapped map and navigate use:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

### Running Nav2
Always Run this on PI first:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Then run these on local pc with different terminals:
```bash
ros2 launch nav2_bringup navigation_launch.py

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml use_sim_time:=false

rviz2
```
Then in rviz, press "add", pick map, then set topic as "/global_costmap/costmap"


## Links

### Den Her Repo
https://github.com/Khanrik/CE-Lab24-Gr1

### Cartographer
Cartographer er bare dårligere

#### Robotis/Turtlebot3 documentation
https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-teleoperation-node

### Nav2
#### Launch Nav2 without prior map
https://answers.ros.org/question/401976/how-to-launch-nav2-without-prior-map/

#### Youtube Easy SLAM
https://www.youtube.com/watch?v=ZaiA3hWaRzE

#### Nav2 SLAM Docs and tutorial
https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html

#### Slam-toolbox
https://github.com/SteveMacenski/slam_toolbox

#### Groot

1: https://navigation.ros.org/tutorials/docs/using_groot.html

2: https://www.behaviortree.dev/docs/ros2_integration

Wrapper: https://github.com/BehaviorTree/BehaviorTree.ROS2?tab=readme-ov-file