#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rrbot_gazebo.srv import MoveToJointPositions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class pd_controller_client(Node):
    def __init__(self):
        """Client for MoveToJointPositions server

        Wait if service not available.
        """

        super().__init__('pd_controller_client')
        self.client = self.create_client(MoveToJointPositions,
                                         'move_to_joint_positions')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = MoveToJointPositions.Request()

    def send_request(self, q_ref, q_measured, curr_time):
        """Return result of request

        Take joint references, the measured joint values and current time.
        Update request, call asynchronously spinning until result received.
        """

        self.request.q_ref = q_ref
        self.request.q_measured = q_measured
        self.request.curr_time = curr_time
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


class pd_controller_pub(Node):
    def __init__(self):
        """Create timer, publisher, client, joint_limit, subscription.

        Initialize q_measured.
        """

        super().__init__('pd_controller_pub')
        self.sample_time = 0.1  # publish every 100ms
        self.timer = self.create_timer(self.sample_time,
                                       self.publish_effort_callback)

        self.publisher_ = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)

        self.client = pd_controller_client()

        self.q_ref = Float64MultiArray()
        self.q_measured = Float64MultiArray()
        self.joint_limit = 0.001

        self.subscription = self.create_subscription(JointState,
                                                     'joint_states',
                                                     self.q_measured_callback,
                                                     10)

    def curr_time(self):
        return time.monotonic()

    def publish_effort_callback(self):
        """If not within joint limits, send request and publish result of
        joint efforts
        """

        self.get_logger().info('TIME: %f' % (self.curr_time()))
        if not self.waiting_for_subscriber():
            if not self.joints_within_limits():
                self.get_logger().info('NOT WITHIN LIMITS')
                self.get_logger().info(
                    'MEAS GIV: q1: %f q2: %f q3: %f' %
                    (self.q_measured.data[0], self.q_measured.data[1],
                     self.q_measured.data[2]))
                result = self.client.send_request(self.q_ref, self.q_measured,
                                                  self.curr_time())
                self.publisher_.publish(result.q_effort)
            else:
                self.get_logger().info('WITHIN LIMITS')

    # ensures we have measured values before publishing
    def waiting_for_subscriber(self):
        return self.q_measured == Float64MultiArray

    # checks that q1, q2, and q3 are "close enough" to goal. can lower limit as controller gets better
    def joints_within_limits(self):
        if (len(self.q_measured.data) > 0):
            self.get_logger().info('ARRAY SIZE %i' % len(self.q_measured.data))
            q1_within_limits = (abs(self.q_ref.data[0] - self.joint_limit) <=
                                abs(self.q_measured.data[0]) <=
                                abs(self.q_ref.data[0] + self.joint_limit))
            q2_within_limits = (abs(self.q_ref.data[1] - self.joint_limit) <=
                                abs(self.q_measured.data[1]) <=
                                abs(self.q_ref.data[1] + self.joint_limit))
            q3_within_limits = (abs(self.q_ref.data[2] - self.joint_limit) <=
                                abs(self.q_measured.data[2]) <=
                                abs(self.q_ref.data[2] + self.joint_limit))
        return q1_within_limits and q2_within_limits and q3_within_limits

    # subscriber callback
    def q_measured_callback(self, msg):
        self.q_measured.data = [
            msg.position[0], msg.position[1], msg.position[2]
        ]


def main(args=None):
    rclpy.init(args=args)

    pd_controller = pd_controller_pub()
    pd_controller.q_ref.data = [
        float(sys.argv[1]),
        float(sys.argv[2]),
        float(sys.argv[3])
    ]
    pd_controller.get_logger().info(
        'REF:  q1: %f q2: %f q3: %f' %
        (pd_controller.q_ref.data[0], pd_controller.q_ref.data[1],
         pd_controller.q_ref.data[2]))

    rclpy.spin(pd_controller)
    pd_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
