#### 运行测试

#### 在RVIZ中显示机器人模型

```
source install/setup.bash
ros2 launch fishbot_description display_rviz2.launch.py
```

#### 仿真
```
source install/setup.bash
ros2 launch fishbot_description gazebo.launch.py
```

#### 建图

```
source install/setup.bash
ros2 launch cartographer_ros my_robot.launch.py 
```


#### Nav2
```
source install/setup.bash
ros2 launch nav2 navigation2.launch.py 
```

#### 启动底盘
```
source install/setup.bash
ros2 launch start start_launch.py 
```

#### 运行导航需要先启动 ros2 launch start start_launch.py 
#### 再启动 ros2 launch nav2 navigation2.launch.py 