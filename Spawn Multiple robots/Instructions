This folder contains the implementation of a swarming algorithm within the Gazebo simulation environment using ROS (Robot Operating System).
The code is designed to support swarming behavior for multiple robots, although it is currently configured for two robots.

Here's an overview of the file structure and key components:

main.launch: This launch file is the entry point. It loads the simulation world and includes another launch file called robots.launch.
robots.launch: This file orchestrates the spawning of robots. It references one_robot.launch twice, with different initial positions and unique names for each robot. Care should be taken to maintain the correct format for initial positions.
one_robot.launch: This launch file sets up a state publisher and spawns a single robot into the Gazebo simulation.
To control each robot individually, you can use the following commands:

```
rosrun key_teleop key_teleop.py robot1:=robot1
rosrun key_teleop key_teleop.py robot2:=robot2
```

To implement the swarming algorithm, terminate the control for robot2 and execute the controller.py script. This script facilitates the swarming behavior, with robot2 following robot1 while maintaining a certain distance.

Terminal commands for execution:
```python
roslaunch myrobot_description main.launch'
rosrun key_teleop key_teleop.py robot1:=robot1
rosrun controller1.py
```
