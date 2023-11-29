#!/usr/bin/env python3

import cmath
import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rrbot_gazebo.srv import MoveToJointPositions


class pd_controller_service(Node):

    def __init__(self):
        """Create a service with custom MoveToJointPositions srv"""

        super().__init__('pd_controller_service')
        self.publisher_ = self.create_publisher(String, 'pos_record', 10)
        self.record = String()
        self.srv = self.create_service(MoveToJointPositions,
                                       'move_to_joint_positions',
                                       self.pd_controller_callback)
        self.Kp = 0.9   # position gain
        self.Kd = 0.01  # derivative gain
        self.last_q_effort = [0.0, 0.0, 0.0]
        self.last_error = [0.0, 0.0, 0.0]
        self.last_time = 0.0

    def pd_controller_callback(self, request, response):
        """Calculate U = Kp * e + Kd * edot for each joint

        Use the measured values provided in the request to calculate
        q_error = q_ref - q_measured. Represent e by q_error. Estimate edot
        with (q_error - previous_q_error) / change in time
        """

        if self.last_time == 0.0:
            self.last_time = request.curr_time
        self.get_logger().info(
            'MEAS:  q1: %f q2: %f q3: %f' %
            (request.q_measured.data[0], request.q_measured.data[1],
             request.q_measured.data[2]))
        # calculate each joint's error from measured to reference
        q_error = [(request.q_ref.data[0] - request.q_measured.data[0]),
                   (request.q_ref.data[1] - request.q_measured.data[1]),
                   (request.q_ref.data[2] - request.q_measured.data[2])]

        dq_error = [(q_error[0] - self.last_error[0]),
                    (q_error[1] - self.last_error[1]),
                    (q_error[2] - self.last_error[2])]

        self.get_logger().info('error:  q1: %f q2: %f q3: %f' %
                               (q_error[0], q_error[1], q_error[2]))
        self.get_logger().info('d_error:  q1: %f q2: %f q3: %f' %
                               (dq_error[0], dq_error[1], dq_error[2]))

        # get the change in time since last sample
        self.dt = request.curr_time - self.last_time
        self.get_logger().info('dt: %f curr: %f last: %f' %
                               (self.dt, request.curr_time, self.last_time))
        # re-set last-time for next function call
        self.last_time = request.curr_time
        # calculate each joint's effort (Kp*error + Kd*(d_error/dt))
        if (self.dt > 0):
            q1_effort = (self.Kp * q_error[0]) + (self.Kd *
                                                  (dq_error[0] / self.dt))
            q2_effort = (self.Kp * q_error[1]) + (self.Kd *
                                                  (dq_error[1] / self.dt))
            q3_effort = (self.Kp * q_error[2]) + (self.Kd *
                                                  (dq_error[2] / self.dt))
        else:
            q1_effort = self.last_q_effort[0]
            q2_effort = self.last_q_effort[1]
            q3_effort = self.last_q_effort[2]

        self.get_logger().info('effort:  q1: %f q2: %f q3: %f' %
                               (q1_effort, q2_effort, q3_effort))
        
        #Give referenced and measured joint values to /pos_record
        self.record.data = '%f,%f,%f,%f,%f,%f' % (request.q_ref.data[0], request.q_measured.data[0],request.q_ref.data[1], request.q_measured.data[1], request.q_ref.data[2], request.q_measured.data[2])
        self.publisher_.publish(self.record)

        response.q_effort.data = [
            q1_effort + request.q_measured.data[0],
            q2_effort + request.q_measured.data[1],
            q3_effort + request.q_measured.data[2]
        ]
        self.last_q_effort = response.q_effort.data
        self.last_error = q_error
        return response


def main(args=None):
    rclpy.init(args=args)
    pd_service = pd_controller_service()    # Call constructor
    rclpy.spin(pd_service)                  # Make service continuously available
    rclpy.shutdown()


if __name__ == '__main__':
    main()
