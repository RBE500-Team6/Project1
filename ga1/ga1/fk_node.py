import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import cmath
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class Fwd_Kin(Node):

    def __init__(self):
        
        self.q = Float64MultiArray
        self.q = [0, 0, 0]
        
        super().__init__('forward_kinematics')
        #self.publisher_ = self.create_publisher(Float64MultiArray, 'forward_position_controller/commands', 10)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'end_pose', 10)
        self.subscription = self.create_subscription(JointState,'joint_states',self.callback,10)
        self.subscription
        
        
        
        
    def callback(self,msg):
        self.pose = Float64MultiArray()
        self.pose.data = [0.0,0.0,0.0]
        L1 = 1
        L2 = 0.5
        L3 = 0.5
        
        
        self.q[0] = msg.position[0]
        self.q[1] = msg.position[1]
        self.q[2] = msg.position[2]
        
        c1 = math.cos(self.q[0])
        s1 = math.sin(self.q[0])
        c2 = math.cos(self.q[1])
        s2 = math.sin(self.q[1])
        
        
        self.pose.data[0] = L1*c1 + L2*c1*c2 - L2*s1*s2
        self.pose.data[1] = L1*s1 + L2*c1*s2 + L2*c2*s1
        self.pose.data[2] = L3 - self.q[2]
        
        self.create_timer(2, self.timed_call)
        
        
    
    def timed_call(self):
        self.publisher_.publish(self.pose)
        self.get_logger().info('q1: %f q2: %f q3: %f' % (self.q[0], self.q[1], self.q[2]))
        self.get_logger().info('x: %f y: %f z: %f' % (self.pose.data[0], self.pose.data[1], self.pose.data[2]))
        #self.destroy_node()
        
def main(args=None):
    rclpy.init(args=args)

    forward_kinematics = Fwd_Kin()
    rclpy.spin(forward_kinematics)

    forward_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
