import rclpy
from rclpy.node import Node
# Detection msg format from ARUCO markers
from aruco_opencv_msgs.msg import ArucoDetection
# Twist msg format to jetbot motors
from geometry_msgs.msg import Twist
import math

# what is minimum distance from the ARUCO marker at which 
# the jetbot should have VMAX
MAXVELDIST = 0.20
VMAX = 0.10
VMIN = 0.07 # if he velocity is less than this, the motors are unable to move

# what is minimum distance from the ARUCO marker at which 
# the jetbot should have VMIN
MINDIST = 0.10

# https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
Kp_linear = 1.

# what is maximum angle to the ARUCO marker at which 
# the jetbot should have OMAX
OMAXANGLE = 25 * math.pi/180.
OMAX = 1.00


# what is minimum angle to the ARUCO marker at which 
# the jetbot should have OMIN
OMINANGLE = 5 * math.pi/180.
# if angular velocity is smaller than this, then motors are unable to move the jetbot
OMIN = 0.50 

Kp_angular = 1.

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
        # We are going to chase this marker
        self.chosen_marker_to_chase = None # Undecided for now

    def on_aruco_detection(self, msg):
        """ This function will be called whenever one-single message of ARUCO
        detection message is received. (Basically all the times repeatedly, as
        long as markers are being detected)"""
        self.get_logger().info('Received: "%s"' % msg.markers)
        if not len(msg.markers):
            self.pub.publish(new_twist(0., 0.))
        else:
            # What if there are multiple markers in the scene
            if self.chosen_marker_to_chase is None:
                # this is the first time we received some detections, pick the
                # first marker as the chosen one.
                self.chosen_marker_to_chase = msg.markers[0].marker_id
                themarker = msg.markers[0]
            else:
                # We know the chosen one, look for it in all detections
                themarker = None
                for m in msg.markers:
                    if m.marker_id == self.chosen_marker_to_chase:
                        themarker = m
                if themarker is not None:
                    # Found it
                    # themarker = msg.markers[0]
                    pass
                else:
                    # Give up. Wont chase anyone else other than the chosen
                    # one
                    self.get_logger().warning("Unable to find marker_id %d" %
                                              self.chosen_marker_to_chase)
                    return

            # Extract the minimum (2D) amount of information, x and z coordinate of
            # the marker
            self.get_logger().info('Position: "%s"' %
                                   themarker.pose.position)
            z = themarker.pose.position.z
            x = themarker.pose.position.x
            # Get the angle between the robot facing
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
