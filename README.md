# Project1
**Part 1**<br />
To run the robot, use the following command within the `Project1/` workspace: 
- Gazebo: `ros2 launch rrbot_gazebo rrbot_world.launch.py`
- Rviz with Joint Controller: `ros2 launch rrbot_description view_robot.launch.py` <br />

![rrbot_in_rviz](https://github.com/WPI-RBE500-Team6/Project1/assets/59891541/496a3d86-7bc8-47db-8ecf-06b181605d49)


**Notes:** <br />
The default lengths of the links are set as:
- L1 = 1.5m
- L2 = 1m
- L3 = 0.5m
- L4 = 0.5m<br />

![image](https://github.com/WPI-RBE500-Team6/Project1/assets/59891541/e5739d9a-1581-4d68-876b-3bb102cf07d1)

**Part 3**

The inverse kinematics server client uses a custom srv that takes a Pose message as a request
and returns three float64 values representing q0, q1, and q2.

Before building, you may need to install dependencies:

`rosdep install -i --from-path src --rosdistro humble -y`

After building, run the server with

```
. install/setup.bash
ros2 run rrbot_gazebo server
```

In another terminal, call the service on the desired end effector position with the following command:

```
. install/setup.bash
ros2 service call /calculate_joints rrbot_gazebo/srv/CalculateJoints "{rrbot_pose: {position: {x: 1.299, y: .75, z: .5}}}"`
```

![Inverse Kinematics Server-Client](https://github.com/WPI-RBE500-Team6/Project1/assets/1744257/5b2a9a91-79aa-4c16-b49f-1144ebcf6cfa)
