# Group1 Fr_am Robot Project
The robotics project of Group1 Friday Mornings

This project attempts to realize a basic ROS2 control node for the Create 3 Educational Robot.
The following goals were realized
1. The robot undocks
2. The robot explores the area for a while
3. The robot returns to the dock and docks again
4. At any time, the robot can be stopped and controlled manually
5. The robot avoids obstacles in its way

# Project setup
First clone the code from this repository into a ROS2 Workspace.
Then, clone the repository group1_interfaces (https://github.com/im2ag-m1-robotics-2025/group1_interfaces.git).

Run
```ssh
colcon build --packages-select group1_interfaces robot_project
```
to build the packages.

# Running the project
After establishing a connection to a Robot, run
```ssh
ros2 run robot_project robot_move_node --ros-args -p topicPrefix:="some prefix"
```
where some prefix is the namespace of the Robot, for example /Robot4

To interrupt the robot, another terminal must be opened. In this one run
```ssh
ros2 topic pub /interrupt_topic group1_interfaces/msg/Interrupt "{}"
```
This will make the node not send any commands to the robot anymore

Type w,a,s or d into the terminal running the node to control the robot.
The mappings are as follows:
- w: move forwards
- s: move backwards
- a: turn counter clockwise
- d: turn clockwise

Alternatively, it is of course possible to use another node to directly communicate with the robot, like teleop_tiwst_keyboard (https://wiki.ros.org/teleop_twist_keyboard) for example.

# Expected behaviour
The Robot explores the area around it for a certain time. If it ever bumps into an obstacle, it will back up a bit and then turn around. After the time is up, the robot will drive back to the dock and dock again.



