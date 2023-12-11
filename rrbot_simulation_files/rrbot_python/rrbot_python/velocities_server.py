#!/usr/bin/env python3

from rrbot_gazebo.srv import CalculateJointVelocities 
from rrbot_gazebo.srv import CalculateEndEffectorVelocity
from numpy import array, linalg, matmul
from math import sin, cos

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CalculateVelocitiesService(Node):

    def __init__(self):
        """Create instance with services to calculate end effector or joint
        velocities.
        """

        super().__init__('calculate_velocities_service')
        self.q1 = float()
        self.q2 = float()
        self.q3 = float()
        self.subscription = self.create_subscription(JointState,
                                                     "joint_states",
                                                     self.joint_states_callback,
                                                     10)
        self.end_effector_service = self.create_service(CalculateEndEffectorVelocity,
                                                        'calculate_end_effector_velocity',
                                                        self.calculate_end_effector_velocity_callback)
        self.joints_service = self.create_service(CalculateJointVelocities,
                                                  'calculate_joint_velocities',
                                                   self.calculate_joint_velocities_callback)

    def joint_states_callback(self, msg):
        """Set q attribute of node with joint positions"""

        self.get_logger().info(f'joint_states_callback: {msg}')
        self.q1 = msg.position[0]
        self.q2 = msg.position[1]
        self.q3 = msg.position[2]
        return

    def jacobian(self):
        """Calculate Jacobian"""

        sigma1 = .5 * sin(self.q1 + self.q2)
        sigma2 = .5 * cos(self.q1 + self.q2)
        return array([
            [-sigma1-sin(self.q1), -sigma1, 0],
            [sigma2-cos(self.q1), sigma2, 0],
            [0, 0, 1],
            [0, 0, 0],
            [0, 0, 0],
            [1, 1, 0]
        ])

    def pseudoinverse(self):
        """Calculate pseudoinverse of the Jacobian"""

        return linalg.pinv(self.jacobian())

    def calculate_end_effector_velocity_callback(self, request, response):
        """Calculate end effector velocity (twist)

        Calculates twist = Jacobian * joint velocities
        """

        self.get_logger().info(f'Request to calculate end effector velocity')
        q = array([
            [self.q1],
            [self.q2],
            [self.q3]
        ])
        twist = matmul(self.jacobian(), q)
        response.twist1 = float(twist[0])
        response.twist2 = float(twist[1])
        response.twist3 = float(twist[2])
        response.twist4 = float(twist[3])
        response.twist5 = float(twist[4])
        response.twist6 = float(twist[5])
        return response

    def calculate_joint_velocities_callback(self, request, response):
        """Calculate joint velocities

        Calculates pseudoinverse of Jacobian * twist = joint_velocities
        """

        self.get_logger().info(f'Request to calculate joint velocities')
        twist = array([
            [request.twist1],
            [request.twist2],
            [request.twist3],
            [request.twist4],
            [request.twist5],
            [request.twist6]
        ])
        q = matmul(self.pseudoinverse(), twist)
        response.q1 = float(q[0])
        response.q2 = float(q[1])
        response.q3 = float(q[2])
        return response

def main():
    rclpy.init()
    calculate_velocities_service = CalculateVelocitiesService()
    rclpy.spin(calculate_velocities_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
