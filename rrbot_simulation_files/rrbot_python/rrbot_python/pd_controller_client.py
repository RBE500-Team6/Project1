#!/usr/bin/env python3

import cmath
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rrbot_gazebo.srv import MoveToJointPositions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String


class pd_controller_client(Node):
    def __init__(self):

        super().__init__('pd_controller_client')
        self.sample_time = 0.1  #publish every 100ms
        self.timer = self.create_timer(self.sample_time,
                                       self.publish_effort_callback)

        self.publisher_ = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)
        self.subscription = self.create_subscription(JointState,
                                                     'joint_states',
                                                     self.q_measured_callback,
                                                     10)
        self.client = self.create_client(MoveToJointPositions,
                                         'move_to_joint_positions')
        self.request = MoveToJointPositions.Request()

        self.q_ref = Float64MultiArray()
        # self.q_ref.data = [0, 0, 0]
        self.q_measured = Float64MultiArray()
        # self.q_measured.data = [0, 0, 0]
        self.joint_limit = float
        self.joint_limit = 0.05
        self.curr_time = Float32
        self.curr_time = 0.0

        # self.Kp = 1.0 #position gain
        # self.Kd = 1.0 #derivative gain

    def send_request(self, q_ref, q_measured):
        self.request.q_ref = q_ref
        self.request.q_measured = q_measured
        self.request.curr_time = time.time()
        self.client.wait_for_service()
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(
            'REF:  q1: %f q2: %f q3: %f' %
            (self.q_ref.data[0], self.q_ref.data[1], self.q_ref.data[2]))
        self.get_logger().info(
            'MEAS: q1: %f q2: %f q3: %f' %
            (self.q1_measured, self.q3_measured, self.q3_measured))
        self.get_logger().info(
            'EFF:  q1: %f q2: %f q3: %f' %
            (future.result.q_effort.data[0], future.result.q_effort.data[1],
             future.result.q_effort.data[2]))

        return self.future.result()

    def q_measured_callback(self, msg):
        self.q1_measured = msg.position[0]
        self.q2_measured = msg.position[1]
        self.q3_measured = msg.position[2]

        self.get_logger().info(
            'MEAS: q1: %f q2: %f q3: %f' %
            (self.q1_measured, self.q3_measured, self.q3_measured))

    def publish_effort_callback(self):
        while (not self.joints_within_limits()):
            self.get_logger().info('NOT WITHIN LIMITS')
            q_measured = Float64MultiArray()
            q_measured.data.append(self.q1_measured)
            q_measured.data.append(self.q2_measured)
            q_measured.data.append(self.q3_measured)
            result = self.send_request(self.q_ref, q_measured)
            self.publisher_.publish(result)
        self.get_logger().info('WITHIN LIMITS')

    def joints_within_limits(self):
        q1_within_limits = ((self.q_ref.data[0] - self.joint_limit) <=
                            self.q1_measured <=
                            (self.q_ref.data[0] + self.joint_limit))
        q2_within_limits = ((self.q_ref.data[1] - self.joint_limit) <=
                            self.q2_measured <=
                            (self.q_ref.data[1] + self.joint_limit))
        q3_within_limits = ((self.q_ref.data[2] - self.joint_limit) <=
                            self.q3_measured <=
                            (self.q_ref.data[2] + self.joint_limit))
        return q1_within_limits and q2_within_limits and q3_within_limits


def main(args=None):
    rclpy.init(args=args)

    pd_client = pd_controller_client()
    pd_client.q_ref.data = [
        float(sys.argv[1]),
        float(sys.argv[2]),
        float(sys.argv[3])
    ]
    pd_client.get_logger().info(
        'REF:  q1: %f q2: %f q3: %f' %
        (pd_client.q_ref.data[0], pd_client.q_ref.data[1],
         pd_client.q_ref.data[2]))

    rclpy.spin(pd_client)
    # if (pd_client.q1_measured and pd_client.q2_measured
    #         and pd_client.q3_measured):
    #     result = pd_client.send_request()

    pd_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
