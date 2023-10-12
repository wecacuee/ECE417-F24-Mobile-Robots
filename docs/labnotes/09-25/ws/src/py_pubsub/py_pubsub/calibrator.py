import rclpy
from rclpy.node import Node

from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
import numpy as np


VMIN = 0.12 # 13 cm per sec
OMIN = np.pi # one rotation per two sec
DT = 0.5
GOAL_MARKER_ID = 7

# https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py
def quaternion_about_axis(angle, axis):
    """Return quaternion for rotation about axis.

    >>> q = quaternion_about_axis(0.123, [1, 0, 0])
    >>> numpy.allclose(q, [0.99810947, 0.06146124, 0, 0])
    True

    """
    q = numpy.array([0.0, axis[0], axis[1], axis[2]])
    qlen = vector_norm(q)
    if qlen > _EPS:
        q *= math.sin(angle / 2.0) / qlen
    q[0] = math.cos(angle / 2.0)
    return q

def axangle_from_quat(quat):
    quat /= np.linalg.norm(quat, axis=-1)
    sin_theta_by_2 = np.linalg.norm(quat[..., 1:4], axis=-1)
    cos_theta_by_2 = quat[..., 0]
    theta = 2 * np.arctan2(sin_theta_by_2, cos_theta_by_2)
    axis = quat[..., 1:5] / sin_theta_by_2
    return axis, theta

def rotmat(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

def shortest_path_astar(state, obstacles, goal):
    # astar algorithm here
    path = [(0.01, 0.01)]
    return path

def new_twist(linear_vel, ang_vel, scale_lin=-0.60, scale_ang=-0.70):
    twist = Twist()
    twist.linear.x = scale_lin * linear_vel
    twist.linear.y = 0.
    twist.linear.z = 0.
    twist.angular.x = 0.
    twist.angular.y = 0.
    twist.angular.z = scale_ang * ang_vel
    return twist

class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrator')
        self.sub = self.create_subscription(ArucoDetection,
                                            '/aruco_detections', 
                                            self.on_aruco_detection, 10)
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.counter = 5
        self.ignore_aruco_detection = False
        self.publish_zero_timer = None
        self.renable_subscriptions_timer = None
        self.marker2pose = {}
        self.prev_robot_pose_wrt_goal = []
        self.rate = self.create_rate(int(1.0/DT))

    def on_aruco_detection(self, msg):
        if self.ignore_aruco_detection:
            return
        for m in msg.markers:
            axis, angle = axangle_from_quat(np.array(
                [m.pose.orientation.w, 
                 m.pose.orientation.x,
                 m.pose.orientation.y,
                 m.pose.orientation.z]))
            self.marker2pose[m.marker_id] = np.array(
                 [m.pose.position.x, m.pose.position.z,
                 angle])
        pose_goal = self.marker2pose[GOAL_MARKER_ID]
        robot_pose_wrt_goal = np.hstack((
            - rotmat(-pose_goal[2]) @ pose_goal[:2],
            - pose_goal[2]))
        self.get_logger().info('Pose: {}'.format(robot_pose_wrt_goal))
        if len(self.prev_robot_pose_wrt_goal):
            self.get_logger().info('Lin vel: {}'.format(
                np.linalg.norm(self.prev_robot_pose_wrt_goal[-1][:2] -
                               robot_pose_wrt_goal[:2]) / DT
            ))
        else:
            if len(self.prev_robot_pose_wrt_goal) > 5:
                sellf.prev_robot_pose_wrt_goal.pop(0)

        self.prev_robot_pose_wrt_goal.append(robot_pose_wrt_goal)
        if self.counter >= 0:
            self.pub.publish(new_twist(VMIN, 0.))
            self.counter -= 1
            self.disable_subscription()
            self.publish_zero_timer = self.create_timer(DT, self.publish_zero)
        else:
            self.disable_subscription()

    def publish_zero(self):
        self.pub.publish(new_twist(0., 0.))
        self.renable_subscriptions_timer = self.create_timer(
            DT, self.renable_subscription)
        self.destroy_timer(self.publish_zero_timer)

    def disable_subscription(self):
        self.ignore_aruco_detection = True

    def renable_subscription(self):
        self.ignore_aruco_detection = False
        self.destroy_timer(self.renable_subscriptions_timer)




def main(args=None):
    rclpy.init(args=args)
    astar = Calibrator()

    rclpy.spin(astar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    astar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
