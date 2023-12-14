#!/usr/bin/env python3

import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rrbot_gazebo.srv import (CalculateEndEffectorVelocity,
                              CalculateJointVelocities)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class pi_controller_client(Node):
    def __init__(self):
        super().__init__('pi_controller_client')

        self.pi_client = self.create_client(CalculateJointVelocities,
                                            'calculate_joint_velocities')
        while not self.pi_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = CalculateJointVelocities.Request()

    def send_request(self, twist1, twist2, twist3, twist4, twist5, twist6):

        self.request.twist1 = twist1
        self.request.twist2 = twist2
        self.request.twist3 = twist3
        self.request.twist4 = twist4
        self.request.twist5 = twist5
        self.request.twist6 = twist6

        future = self.pi_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


class pi_controller_pub(Node):
    def __init__(self):
        super().__init__('pi_controller_pub')
        self.ts = 0.1  #10ms
        self.timer = self.create_timer(self.ts, self.publish_effort_callback)
        self.publisher_ = self.create_publisher(
            Float64MultiArray, "/forward_velocity_controller/commands", 10)
        self.test = self.create_publisher(Float64MultiArray, "/test", 10)

        self.pi_client = pi_controller_client()
        self.pub_effort = Float64MultiArray()
        self.qvel_measured = Float64MultiArray()
        self.qvel_measured.data = [0.0, 0.0, 0.0]
        self.qvel_ref = Float64MultiArray()
        self.qvel_ref.data = [0.0, 0.0, 0.0]
        self.last_qv_effort = [0.0, 0.0, 0.0]
        self.twist = Float64MultiArray()
        self.last_time = 0.0
        self.joint_limit = 0.001
        self.integral1 = 0.0
        self.integral2 = 0.0
        self.integral3 = 0.0

        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.qvel_measured_callback, 10)

    def curr_time(self):
        return time.monotonic()

    def publish_effort_callback(self):

        Ki = 0.1
        Kp = 0.2
        twist1 = self.twist.data[0]
        twist2 = self.twist.data[1]
        twist3 = self.twist.data[2]
        twist4 = self.twist.data[3]
        twist5 = self.twist.data[4]
        twist6 = self.twist.data[5]

        if self.last_time == 0.0:
            self.last_time = time.monotonic()

        self.get_logger().info('TIME: %f' % (self.curr_time()))
        if not self.waiting_for_subscriber():
            if not self.joints_within_limits():
                self.get_logger().info('NOT WITHIN LIMITS')
                self.get_logger().info(
                    'MEAS GIV: v1: %f v2: %f v3: %f' %
                    (self.qvel_measured.data[0], self.qvel_measured.data[1],
                     self.qvel_measured.data[2]))
                result = self.pi_client.send_request(twist1, twist2, twist3,
                                                     twist4, twist5, twist6)
                self.qvel_ref.data[0] = result.q1
                self.qvel_ref.data[1] = result.q2
                self.qvel_ref.data[2] = result.q3

                err = [(self.qvel_ref.data[0] - self.qvel_measured.data[0]),
                       (self.qvel_ref.data[1] - self.qvel_measured.data[1]),
                       (self.qvel_ref.data[2] - self.qvel_measured.data[2])]
                self.c_time = time.monotonic()
                self.dt = self.c_time - self.last_time
                self.last_time = time.monotonic()

                self.integral1 += (Ki * err[0] * self.dt)
                self.integral2 += (Ki * err[1] * self.dt)
                self.integral3 += (Ki * err[2] * self.dt)

                if self.dt > 0:
                    qv1_effort = ((Kp * err[0]) + self.integral1)
                    qv2_effort = ((Kp * err[1]) + self.integral2)
                    qv3_effort = ((Kp * err[2]) + self.integral3)
                else:
                    qv1_effort = self.last_qv_effort[0]
                    qv2_effort = self.last_qv_effort[1]
                    qv3_effort = self.last_qv_effort[2]

                self.last_err = err

                self.pub_effort.data = [qv1_effort, qv2_effort, qv3_effort]
                self.publisher_.publish(self.pub_effort)
                #self.get_logger().info('qvm1 size: %f' % (len(self.qvel_measured.data)))
                ref1 = self.qvel_ref.data[0]
                ref2 = self.qvel_ref.data[1]
                ref3 = self.qvel_ref.data[2]

                #self.get_logger().info('qvm1: %f' % (self.qvel_ref.data[1]))
                #self.get_logger().info('int1: %f' % (Kp*err[0]))
                #self.get_logger().info('qv_meas3: %f' % (self.qvel_measured.data[2]))
                #self.get_logger().info('===PUBLISHED EFFORT: %f %f %f' % (self.qv_effort.data[0],
                #self.qv_effort.data[1], self.qv_effort.data[2]))
                self.last_qv_effort = self.pub_effort.data
                self.check = 1
            else:
                self.get_logger().info('WITHIN LIMITS')

    def waiting_for_subscriber(self):
        return self.qvel_measured == Float64MultiArray

    def joints_within_limits(self):

        qvm = self.qvel_measured.data
        qvr = self.qvel_ref.data

        q1_within_limits = (abs(qvr[0] - self.joint_limit) <= abs(qvm[0]) <=
                            abs(qvr[0] + self.joint_limit))
        q2_within_limits = (abs(qvr[1] - self.joint_limit) <= abs(qvm[1]) <=
                            abs(qvr[1] + self.joint_limit))
        q3_within_limits = (abs(qvr[2] - self.joint_limit) <= abs(qvm[2]) <=
                            abs(qvr[2] + self.joint_limit))

        return q1_within_limits and q2_within_limits and q3_within_limits

    def qvel_measured_callback(self, msg):

        vm = [0.0, 0.0, 0.0]
        vm = msg.velocity
        self.qvel_measured.data = vm
        #self.test.publish(self.qvel_measured)
        "The length of the measured velocity array will briefly go to 0, reason or effects are unclear"
        self.get_logger().info('qvm1 size: %f' %
                               (len(self.qvel_measured.data)))

    def publish_data(self, msg):
        #effort = msg.data
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    pi_controller = pi_controller_pub()
    pi_controller.twist.data = [
        float(sys.argv[1]),
        float(sys.argv[2]),
        float(sys.argv[3]),
        float(sys.argv[4]),
        float(sys.argv[5]),
        float(sys.argv[6])
    ]
    pi_controller.get_logger().info(
        'REF:  tw1: %f tw2: %f tw3: %f tw4: %f tw5: %f tw6: %f' %
        (pi_controller.twist.data[0], pi_controller.twist.data[1],
         pi_controller.twist.data[2], pi_controller.twist.data[3],
         pi_controller.twist.data[4], pi_controller.twist.data[5]))

    rclpy.spin(pi_controller)
    pi_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
