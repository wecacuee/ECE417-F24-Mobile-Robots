import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
import math

MAXVELDIST = 0.20
VMAX = 0.10

VMIN = 0.07
MINDIST = 0.10
Kp_linear = 1.

OMIN = 0.50
OMINANGLE = 5 * math.pi/180.

OMAX = 1.00
OMAXANGLE = 25 * math.pi/180.

Kp_angular = 1.

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
    return twist



class MoveToARUCO(Node):
    def __init__(self):
        super().__init__('move2aruco')
        self.sub = self.create_subscription(ArucoDetection,
        '/aruco_detections',
        self.on_aruco_detection, 10)
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.timer = None
        self.last_marker_id = None

    def on_aruco_detection(self, msg):
        """ This function will be called whenever an ARUCO detection message 
        is received"""
        self.get_logger().info('Received: "%s"' % msg.markers)
        if len(msg.markers):
            self.get_logger().info('Position: "%s"' %
                                   msg.markers[0].pose.position)
            # Extract the minimum amount of information, x and z coordinate of
            # the marker
            marker_position_z = msg.markers[0].pose.position.z
            z = marker_position_z
            x = msg.markers[0].pose.position.x
            # Recall trignometry and get the angle between the robot facing
            # direction and the line connecting the robot to the marker
            angle = math.atan2(x, z)

            # Do linear interpolation for linear velocity
            linear_vel = (VMIN
                          + Kp_linear * (VMAX - VMIN) * (z - MINDIST)
                          / (MAXVELDIST - MINDIST))
            # Clip the linear velocity between VMIN and VMAX within safe
            # limits
            linear_vel_clipped = (VMAX if  abs(VMAX) < abs(linear_vel) else
                                  0. if abs(linear_vel) < abs(VMIN) else
                                  linear_vel)
            self.get_logger().info('Linear: "%f";' % linear_vel_clipped)

            # Do linear interpolation for angular velocity
            ang_vel = (OMIN + Kp_angular * (OMAX - OMIN) * (angle - OMINANGLE) /
                       (OMAXANGLE - OMINANGLE))
            # Clip the angular velocity between OMAX and OMIN within safe
            # limits
            ang_vel_clipped = (OMAX if abs(OMAX) < abs(ang_vel) else
                               0. if abs(ang_vel) < abs(OMIN) else
                               ang_vel)
            self.get_logger().info('Angular: "%f";' % ang_vel_clipped)
            self.pub.publish(new_twist(-linear_vel_clipped, ang_vel_clipped))
        else:
            self.pub.publish(new_twist(0., 0.))


def main(args=None):
    rclpy.init(args=args)

    move2a = MoveToARUCO()

    rclpy.spin(move2a)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move2a.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
