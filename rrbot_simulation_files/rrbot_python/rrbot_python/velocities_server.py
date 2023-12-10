#!/usr/bin/env python3

from rrbot_gazebo.srv import CalculateJointVelocities 
from rrbot_gazebo.srv import CalculateEndEffectorVelocity
from numpy import array, linalg, matmul
from math import sin, cos

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CalculateVelocitiesService(Node):

    def __init__(self):
        """Create instance with services to calculate end effector or joint
        velocities.
        """

        super().__init__('calculate_velocities_service')
        self.q = Float64MultiArray()
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
        self.q.data = [
            msg.position[0],
            msg.position[1],
            msg.position[2]
        ]
        return

    def jacobian(self):
        """Calculate Jacobian"""

        sigma1 = .5 * sin(self.q.data[0] + self.q.data[1])
        sigma2 = .5 * cos(self.q.data[0] + self.q.data[1])
        return array([
            [-sigma1-sin(self.q.data[0]), -sigma1, 0],
            [sigma2-cos(self.q.data[0]), sigma2, 0],
            [0, 0, 0],
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
        msg = ('Request\nq1: {request.q.data[0]} q2: {request.q.data[1]} '
               'q3: {request.q.data[2]}')
        self.get_logger().info(msg)
        q = array([
            [self.q.data[0]],
            [self.q.data[1]],
            [self.q.data[2]]
        ])
        twist = matmul(self.jacobian(), q)
        response.twist.data = [
            float(twist[0]),
            float(twist[1]),
            float(twist[2]),
            float(twist[3]),
            float(twist[4]),
            float(twist[5])
        ]
        msg = ('Response\ntwist: {response.twist.data[0]}, '
               '{response.twist.data[1]} '
               '{response.twist.data[2]} '
               '{response.twist.data[3]} '
               '{response.twist.data[4]} '
               '{response.twist.data[5]} '
               '{response.twist.data[6]}')
        self.get_logger().info(msg)
        return response

    def calculate_joint_velocities_callback(self, request, response):
        """Calculate joint velocities

        Calculates pseudoinverse of Jacobian * twist = joint_velocities
        """

        self.get_logger().info(f'Request to calculate joint velocities')
        self.get_logger().info('Request: twist = [')
        for value in twist.data:
            self.get_logger().info(f'{value}\n')
        self.get_logger().info(']\n')
        self.get_logger().info(f'q: {self.q.data}')
        twist = array([
            [request.twist.data[0]],
            [request.twist.data[1]],
            [request.twist.data[2]],
            [request.twist.data[3]],
            [request.twist.data[4]],
            [request.twist.data[5]]
        ])
        q = mulmat(self.pseudoinverse(), twist)
        response.q.data[0] = q[0]
        response.q.data[1] = q[1]
        response.q.data[2] = q[2]
        return response

def main():
    rclpy.init()
    calculate_velocities_service = CalculateVelocitiesService()
    rclpy.spin(calculate_velocities_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
