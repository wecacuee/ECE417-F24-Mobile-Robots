import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist

VMIN = 0.07
OMIN = 1.25
DT = 0.1

def new_twist(linear_vel, ang_vel):
    """Generate a new command to be sent to the motors_waveshare

    linear_vel: linear velocity
    ang_vel : angular velocity
    """
    twist = Twist()
    twist.linear.x = linear_vel
    twist.linear.y = 0.
    twist.linear.z = 0.
    twist.angular.x = 0.
    twist.angular.y = 0.
    twist.angular.z = ang_vel



class MoveToARUCO(Node):
    def __init__(self):
        super().__init__('astar')
        self.sub = self.create_subscription(ArucoDetection,
        '/aruco_detections',
        self.on_aruco_detection, 10)
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.timer = None

    def on_aruco_detection(self, msg):
        """ This function will be called whenever an ARUCO detection message 
        is received"""
        self.get_logger().info('Received: "%s"' % msg.markers)

def main(args=None):
    rclpy.init(args=args)

    move2a = MoveToARUCO()

    rclpy.spin(astar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move2a.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
