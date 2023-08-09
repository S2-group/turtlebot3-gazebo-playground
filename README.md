# turtlebot3-gazebo-playground

## 1. install ros2-humble, gazebo, navigation2, cartographer

## 2. add and launch new launch files:

aws world launch file:
```console
roy@roy-ubuntu22:/opt/ros/humble/share/turtlebot3_gazebo/launch$ ros2 launch turtlebot3_gazebo turtlebot3_house_aws.launch.py
```

cartographer launch file
```console
roy@roy-ubuntu22:/opt/ros/humble/share/turtlebot3_cartographer/launch$ ros2 launch turtlebot3_cartographer cartographer_aws_rviz.launch.py use_sim_time:=True
```

navigation2 launch file
```console
roy@roy-ubuntu22:/opt/ros/humble/share/turtlebot3_navigation2/launch$ ros2 launch turtlebot3_navigation2 navigation2_aws_rviz.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

publish destination
```console
ros2 action send_goal navigate_to_pose nav2_msgs/NavigateToPose "pose: { pose: {position: {x: 3.4, y: 0.8, z: 0}} }"
```

## 3. run message-stats
### listen to all topics and record
build
```console
colcon build --packages-select message_stats
source install/setup.bash
ros2 run message_stats all
```

