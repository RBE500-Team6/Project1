#!/usr/bin/env python3

import cmath
import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rrbot_gazebo.srv import MoveToJointPositions
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String


class pd_controller_service(Node):
    def __init__(self):

        super().__init__('pd_controller')
        self.srv = self.create_service(MoveToJointPositions,
                                       'move_to_joint_positions',
                                       self.pd_controller_callback)

        self.Kp = 1.0  #position gain
        self.Kd = 1.0  #derivative gain
        # self.last_q_effort = [0.0, 0.0, 0.0]

        #self.last_time = 0.0

    def pd_controller_callback(self, request, response):
        # calculate each joint's error from measured to reference
        q_error = [(request.q_ref.data[0] - request.q_measured.data[0]),
                   (request.q_ref.data[1] - request.q_measured.data[1]),
                   (request.q_ref.data[2] - request.q_measured.data[2])]

        # Not part of request; we could change the srv to include it or use the
        # same value?
        # get the change in time since last sample
        #self.dt = request.curr_time - self.last_time
        self.dt = 0.01 

        # calculate each joint's effort (Kp*error + Kd*(error/dt))
        q1_effort = (self.Kp * q_error[0]) + (self.Kd * (q_error[0] / self.dt))
        q2_effort = (self.Kp * q_error[1]) + (self.Kd * (q_error[1] / self.dt))
        q3_effort = (self.Kp * q_error[2]) + (self.Kd * (q_error[2] / self.dt))
        response = Float64MultiArray(data=[q1_effort, q2_effort, q3_effort])

        # re-set last-time for next function call
        #self.last_time = request.curr_time
        return response


def main():
    rclpy.init()

    pd_service = pd_controller_service()
    rclpy.spin(pd_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
