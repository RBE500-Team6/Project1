#!/usr/bin/env python3

import cmath
import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rrbot_gazebo.srv import MoveToJointPositions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String


class pd_controller_client(Node):

    def __init__(self, data):

        super().__init__('pd_controller_client')
        self.sample_time = 0.1  #publish every 100ms
        self.timer = self.create_timer(self.sample_time,
                                       self.publish_effort_callback)
        self.rate = self.create_rate(100)

        self.q_ref = Float64MultiArray(data=data)
        self.q_measured = Float64MultiArray(data=[0.0, 0.0, 0.0])
        self.publisher_ = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)
        self.subscription = self.create_subscription(JointState,
                                                     'joint_states',
                                                     self.q_measured_callback,
                                                     10)
        self.client = self.create_client(MoveToJointPositions,
                                         'move_to_joint_positions')
        self.request = MoveToJointPositions.Request()

        self.joint_limit = 0.05

        # self.Kp = 1.0 #position gain
        # self.Kd = 1.0 #derivative gain

    def send_request(self):
        self.request.q_ref = self.q_ref
        self.request.q_measured = self.q_measured
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def q_measured_callback(self, msg):
        data = [msg.position[0], msg.position[1], msg.position[2]]
        self.q_measured = Float64MultiArray(data=data)
        return self.q_measured

    def publish_effort_callback(self):
        result = self.send_request()
        self.publisher_.publish(result)
        self.get_logger().info(
            'REF:  q1: %f q2: %f q3: %f' %
            (self.request.q_ref.data[0], self.request.q_ref.data[1],
             self.request.q_ref.data[2]))
        self.get_logger().info(
            'MEAS: q1: %f q2: %f q3: %f' %
            (self.q_measured.data[0], self.q_measured.data[1],
             self.q_measured.data[2]))
        self.get_logger().info('EFF:  q1: %f q2: %f q3: %f' %
                               (result[0], result[1], result[2]))

    def joints_within_limits(self):
        q1_within_limits = ((self.q_measured.data[0] <=
                             (self.request.q_ref.data[0] + self.joint_limit))
                            and (self.q_measured.data[0] >=
                                 (self.request.q_ref.data[0] - self.joint_limit)))
        q2_within_limits = ((self.q_measured.data[1] <=
                             (self.request.q_ref.data[1] + self.joint_limit))
                            and (self.q_measured.data[1] >=
                                 (self.request.q_ref.data[1] - self.joint_limit)))
        q3_within_limits = ((self.q_measured.data[2] <=
                             (self.request.q_ref.data[2] + self.joint_limit))
                            and (self.q_measured.data[2] >=
                                 (self.request.q_ref.data[2] - self.joint_limit)))
        return q1_within_limits and q2_within_limits and q3_within_limits


def parse_data(data):
    """Parse '{data: {a, b, c}}' for floats a, b, c"""

    nums = data.split(":")[1].strip(" {}").split(",")
    return [float(num.strip(" ")) for num in nums]

def main(args=None):
    rclpy.init(args=args)

    pd_client = pd_controller_client(data=parse_data(sys.argv[1]))
    #pd_client.q_ref = (sys.argv[1])
    response = pd_client.send_request()

    #rclpy.spin(pd_client)
    while (not pd_client.joints_within_limits()):
        rclpy.spin_once(pd_client)
        pd_client.rate.sleep()

    pd_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
