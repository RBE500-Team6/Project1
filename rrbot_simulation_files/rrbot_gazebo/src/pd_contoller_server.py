import cmath
import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from srv import MoveToJointPositions
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String


class pd_controller_service(Node):
    def __init__(self):

        super().__init__('pd controller')
        self.srv = self.create_service(MoveToJointPositions,
                                       'move_to_joint_positions',
                                       self.pd_controller_callback)

        self.Kp = 1.0  #position gain
        self.Kd = 1.0  #derivative gain
        # self.last_q_effort = [0.0, 0.0, 0.0]

        self.last_time = 0.0

    def pd_controller_callback(self, request, response):
        # calculate each joint's error from measured to reference
        q_error = [(request.q_ref[0] - request.q_measured[0]),
                   (request.q_ref[1] - request.q_measured[1]),
                   (request.q_ref[2] - request.q_measured[2])]

        # get the change in time since last sample
        self.dt = request.curr_time - self.last_time

        # calculate each joint's effort (Kp*error + Kd*(error/dt))
        q1_effort = (self.Kp * q_error[0]) + (self.Kd * (q_error[0] / self.dt))
        q2_effort = (self.Kp * q_error[1]) + (self.Kd * (q_error[1] / self.dt))
        q3_effort = (self.Kp * q_error[2]) + (self.Kd * (q_error[2] / self.dt))
        response = [q1_effort, q2_effort, q3_effort]

        # re-set last-time for next function call
        self.last_time = request.curr_time
        return response


def main(args=None):
    rclpy.init(args=args)

    pd_controller_service = pd_controller_service()
    rclpy.spin(pd_controller_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
