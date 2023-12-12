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

# Project 2

Place packages in `rrbot_simulation_files` in your ros2 workspace then build it.

Install dependencies if needed

    rosdep install --from-paths src -y --ignore-src

Install packages

    colcon build --packages-select rrbot_gazebo rrbot_description rrbot_python

Run `rrbot_python`. In the same terminal

    . install/setup.bash
    ros2 launch rrbot_gazebo rrbot_world.launch.py

In another terminal, run the service:

    . install/setup.bash
    ros2 run rrbot_python service

In another terminal, run the controller:

    . install/setup.bash
    ros2 run rrbot_python pd_client 0 0 0.5

In a separate terminal, echo the `/forward_position_controller/commands`:

    . install/setup.bash
    ros2 topic echo /forward_position_controller/commands

# Project 3

1. Before using the node, install numpy

    pip3 install numpy

A node with two services has been added to rrbot_python:
   * CalculateEndEffectorVelocity
   * CalculateJointVelocities

After compiling rrbot_python, you may wish to launch rviz to manipulate joints since rrbot_python's
velocities_service subscribes to /joint_states.

    # Launch rviz to move joints if desired
    ros2 launch rrbot_description view_robot.launch.py
    # Launch velocity service
    ros2 run rrbot_python velocities_service

Make calls to the desired service as follows:

    ros2 service call /calculate_end_effector_velocity rrbot_gazebo/srv/CalculateEndEffectorVelocity "{q1: 100.0, q2: 150.0, q3: 50.0}"
    ros2 service call /calculate_joint_velocities rrbot_gazebo/srv/CalculateJointVelocities "{twist1: -102.29722673047766, twist2: -84.22055273104294, twist3: 50.0, twist4: 0.0, twist5: 0.0, twist6: 250.0}"

numpy is used to calculate the pseudoinverse of the Jacobian and multiply the
matrices:

    def jacobian():
        """Calculate Jacobian"""

        sigma1 = .5 * sin(q[0] + q[1])  # q[0] represents joint q1
        sigma2 = .5 * cos(q[0] + q[1])  # q[1] represents joint q2
        return array([
            [-sigma1-sin(q[0]), -sigma1, 0],
            [sigma2-cos(q[0]), sigma2, 0],
            [0, 0, 1],
            [0, 0, 0],
            [0, 0, 0],
            [1, 1, 0]
        ])

    def pseudoinverse(self):
        """Calculate pseudoinverse of the Jacobian"""

        return linalg.pinv(self.jacobian())

Which are used to calculate the values returned from the services:
* twist  = jacobian * joint_velocities
* pseudoinverse * twist = joint_velocities

The twist will give us the velocity of the body or end effector velocity.
