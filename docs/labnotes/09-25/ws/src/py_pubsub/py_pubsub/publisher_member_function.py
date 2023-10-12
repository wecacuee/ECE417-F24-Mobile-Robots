import rclpy
from rclpy.node import Node

from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
import numpy as np

VMIN = 0.07
OMIN = np.pi # one rotation per two sec
DT = 0.2

def shortest_path_astar(state, obstacles, goal):
    # astar algorithm here
    path = [(0.01, 0.01)]
    return path

def new_twist(linear_vel, ang_vel, scale_lin=1.0, scale_ang=0.70):
    twist = Twist()
    twist.linear.x = scale_lin * linear_vel
    twist.linear.y = 0.
    twist.linear.z = 0.
    twist.angular.x = 0.
    twist.angular.y = 0.
    twist.angular.z = scale_ang * ang_vel
    return twist

class Astar(Node):
    def __init__(self):
        super().__init__('astar')
        self.sub = self.create_subscription(ArucoDetection,
                                            '/aruco_detections', 
                                            self.on_aruco_detection, 10)
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.counter = 0
        self.next_pub_zero = False
        self.timer = None

    def on_aruco_detection(self, msg):
        if self.timer is None:
            self.pub.publish(new_twist(0., OMIN))
            self.next_pub_zero = True
            self.counter += 1
            self.timer = self.create_timer(DT, self.timer_callback)

    def timer_callback(self):
        if self.next_pub_zero:
            self.pub.publish(new_twist(0., 0.))
            self.next_pub_zero = False
            self.timer.reset()
        elif self.counter < (2/DT):
            self.pub.publish(new_twist(0., OMIN))
            self.next_pub_zero = True
            self.counter += 1
            self.timer.reset()


def main(args=None):
    rclpy.init(args=args)

    print('init astar')
    astar = Astar()

    rclpy.spin(astar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    astar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
