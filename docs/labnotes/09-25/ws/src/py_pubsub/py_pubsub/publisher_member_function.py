import rclpy
from rclpy.node import Node

from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist

VMIN = 0.07
OMIN = 1.25
DT = 0.25

def shortest_path_astar(state, obstacles, goal):
    # astar algorithm here
    path = [(0.01, 0.01)]
    return path

class Astar(Node):

    def __init__(self):
        super().__init__('astar')
        self.get_logger().info('in init')
        self.sub = self.create_subscription(ArucoDetection,
                                            '/aruco_detections', 
                                            self.on_aruco_detection, 10)
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.timer = None

    def on_aruco_detection(self, msg):
        print("hi")
        self.get_logger().info('Received: "%s"' % msg.markers)

        # your fancy Astar algorithm here
        state = None # What should be state?
        obstacles = None # Where are the obstacles?
        goal = None # Where is the goal?
        path = shortest_path_astar(state, obstacles, goal)
        # Take the first step in the path as the action for now
        # action = path[0] # or something else?
        linear_vel, ang_vel = 0.0, OMIN

        # initialize the twist message and publish it the jetbot
        # self.get_logger().info('Publishing: "%s"' % action)
        twist = Twist()
        twist.linear.x = linear_vel
        twist.linear.y = 0.
        twist.linear.z = 0.
        twist.angular.x = 0.
        twist.angular.y = 0.
        twist.angular.z = ang_vel
        if self.timer is None:
            self.pub.publish(twist)
            self.timer = self.create_timer(DT, self.timer_callback)

    def timer_callback(self):
        # initialize the twist message and publish it the jetbot
        twist = Twist()
        twist.linear.x = 0.
        twist.linear.y = 0.
        twist.linear.z = 0.
        twist.angular.x = 0.
        twist.angular.y = 0.
        twist.angular.z = 0.
        self.pub.publish(twist)
        self.destroy_timer(self.timer)


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
