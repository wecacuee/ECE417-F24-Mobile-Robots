import math
from random import random

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path

import rclpy
from rclpy.node import Node
# Detection msg format from ARUCO markers
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
# Twist msg format to jetbot motors
from geometry_msgs.msg import Twist, Quaternion

from move2aruco import MoveToARUCO, PrintLogger, new_twist

def trans_inv(T, D=3):
    Tinv = np.eye(D+1)
    Tinv[:D, :D] = T[:D, :D].T
    Tinv[:D, D] = - T[:D, :D].T @ T[:D, D]
    return Tinv

def transformation_matrix2D(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

def xytheta_from_trans_mat2D(T):
    return T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])


def cross_prod_mat(k):
    return np.array([
        [0., -k[2], k[1]],
        [k[2], 0., -k[0]],
        [-k[1], k[0], 0]
        ])

def vec_from_cross_prod_mat(K):
    assert np.allclose(np.diag(K), np.zeros(3))
    return np.array([-K[1, 2],
                     K[0, 2],
                     -K[0, 1]])


def rodrigues(k, theta):
    k = k / np.linalg.norm(k)
    K = cross_prod_mat(k)
    return np.eye(3) + np.sin(theta) * K + (1-np.cos(theta)) * (K @ K)

class Rot:
    @staticmethod
    def from_quat(quat):
        k = np.array([quat.x, quat.y, quat.z])
        sinthetaby2 = np.linalg.norm(k)
        k = k / sinthetaby2 
        costhetaby2 = quat.w
        theta = 2*np.arctan2(sinthetaby2, costhetaby2)
        return rodrigues(k, theta)

    @staticmethod
    def to_quat(R):
        theta = np.arccos((np.trace(R)-1)/2)
        if np.allclose(R, R.T):
            k = np.array([
                                     np.sqrt((R[0, 0]+1)/2),
                    np.sign(R[0, 1])*np.sqrt((R[1, 1]+1)/2),
                    np.sign(R[0, 2])*np.sqrt((R[2, 2]+1)/2)
                    ])
            assert np.allclose(np.linalg.norm(k), 1)
        else:
            k = vec_from_cross_prod_mat((R - R.T)) / (2*np.sin(theta))
            assert np.allclose(np.linalg.norm(k), 1)

        quat = Quaternion()
        quat.w = np.cos(theta/2)
        quat.x, quat.y, quat.z = k * np.sin(theta/2)
        return quat

def test_rotation_matrix():
    k = np.array([0, 0, 1.])
    theta = np.pi/6
    quat = Quaternion()
    quat.w = np.cos(theta/2)
    quat.x, quat.y, quat.z = k * np.sin(theta/2)
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0.],
        [np.sin(theta),  np.cos(theta), 0.],
        [0,  0, 1.]])
    assert np.allclose(R, Rot.from_quat(quat)), \
            "{} <=> {}".format(R, Rot.from_quat(quat))
    quat_rec = Rot.to_quat(R)
    assert np.allclose(quat_rec.w, quat.w)
    assert np.allclose(quat_rec.x, quat.x)
    assert np.allclose(quat_rec.y, quat.y)
    assert np.allclose(quat_rec.z, quat.z), \
            "{} <=> {}".format(quat_rec, quat)

    k = np.random.rand(3)
    k = k / np.linalg.norm(k)
    theta = (np.random.rand() * 2*np.pi) - np.pi
    quat = Quaternion()
    quat.w = np.cos(theta/2)
    quat.x, quat.y, quat.z = k * np.sin(theta/2)
    R = rodrigues(k, theta)
    assert np.allclose(R, Rot.from_quat(quat)), \
            "{}: {} <=> {}".format(quat, R, Rot.from_quat(quat))
    quat_rec = Rot.to_quat(R)
    assert np.allclose(quat_rec.w, quat.w)
    assert np.allclose(quat_rec.x, quat.x)
    assert np.allclose(quat_rec.y, quat.y)
    assert np.allclose(quat_rec.z, quat.z), \
            "{} <=> {}".format(quat_rec.z, quat.z)


def plot_marker(ax, marker_id, marker_pose):
    verts = np.array([ (-.5, -.5),
                      (-.5, .5),
                      (.5, .5),
                      (.5, -.5),
                      (-.5, -.5)])
    x, y, theta = marker_pose
    T = transformation_matrix2D(x, y, theta)
    verts = verts @ T[:2, :2].T + T[:2, 2]
    path = Path(verts)
    patch = patches.PathPatch(path, facecolor='orange', lw=2)
    mean_verts = np.mean(verts, axis=0)
    ax.text(x=mean_verts[0], y=mean_verts[1], s='%d' % marker_id)
    ax.add_patch(patch)

def plot_vehicle(ax, x, y, theta, x_traj, y_traj, dt):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix2D(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    ax.plot(x_traj, y_traj, 'b--')

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)


class ARUCOSimulator:
    def __init__(self, start, goal, marker_list,
                 dt = 0.01,
                 # Robot specifications
                 MAX_LINEAR_SPEED = 15,
                 MAX_ANGULAR_SPEED = 7,
                 show_animation = True
                 ):
        self.state = start
        self.goal = goal
        self.markers = marker_list
        self.show_animation = True

        self.camera_fov = 80*np.pi/180
        self.dt = dt
        # Robot specifications
        self.MAX_LINEAR_SPEED = MAX_LINEAR_SPEED
        self.MAX_ANGULAR_SPEED = MAX_ANGULAR_SPEED
        self.show_animation = show_animation
        self.pos_traj = []
        self.pos_traj.append(self.state)
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')

    def timer_callback(self):
        x_t, y_t, theta_t = self.state
        x_goal, y_goal, theta_goal = self.goal
        if self.show_animation:  # pragma: no cover
            plt.cla()
            plt.arrow(x_t, y_t, np.cos(theta_t),
                      np.sin(theta_t), color='r', width=0.1)
            plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=0.1)
            plot_vehicle(self.ax, x_t, y_t, theta_t,
                         [p[0] for p in self.pos_traj],
                         [p[1] for p in self.pos_traj],
                        dt=self.dt)
            for marker_id, marker_pose in self.markers.items():
                plot_marker(self.ax, marker_id, marker_pose)
        plt.pause(self.dt)
        msg = ArucoDetection()
        view_dir_t = np.array([np.cos(theta_t), np.sin(theta_t)])
        for marker_id, marker_pose in self.markers.items():
            x_m, y_m, theta_m = marker_pose
            dir_m = np.array([x_m - x_t,y_m - y_t])
            dir_m = dir_m / np.linalg.norm(dir_m)
            # The marker is only viewable within camera FOV
            if view_dir_t @ dir_m > np.cos(self.camera_fov):
                marker_pose = MarkerPose()
                marker_pose.marker_id = marker_id
                T_world_from_mrkr = transformation_matrix2D(x_m, y_m, theta_m)
                T_world_from_cam = transformation_matrix2D(x_t, y_t, theta_t)
                T_cam_from_mrkr = trans_inv(T_world_from_cam, D=2) @ T_world_from_mrkr
                x_m_wrt_cam, y_m_wrt_cam, theta_m_wrt_cam = \
                xytheta_from_trans_mat2D(T_cam_from_mrkr)
                # Assume marker xyz similar to camera coordinate
                # system
                marker_pose.pose.position.x = y_m_wrt_cam
                marker_pose.pose.position.y = 0.
                marker_pose.pose.position.z = x_m_wrt_cam
                marker_pose.pose.orientation.w =  np.cos(
                        theta_m_wrt_cam / 2)
                marker_pose.pose.orientation.x =  0.
                marker_pose.pose.orientation.y = -1. * np.sin(
                        theta_m_wrt_cam / 2)
                marker_pose.pose.orientation.z =  0.
                msg.markers.append(marker_pose)
        return msg

    def on_twist_detection(self, msg):
        x_t, y_t, theta_t = self.state
        self.state = [x_t + msg.linear.x * np.cos(theta_t) * self.dt,
                      y_t + msg.linear.x * np.sin(theta_t) * self.dt,
                      theta_t + msg.angular.z * self.dt]
        self.pos_traj.append(self.state)

class RandomController:
    def __init__(self, m, minu, maxu):
        self.m = m
        self.minu = minu
        self.maxu = maxu
    def control(self, state, state_goal):
        return np.random.rand(self.m) * (self.maxu - self.minu) + self.minu


class ARUCOSimulatorNode(Node):
    def __init__(self, *args, **kw):
        super().__init__('aruco_sim')
        self.sim = ARUCOSimulator(*args, **kw)

        self.pub = self.create_publisher(
                ArucoDetection, '/aruco_detections', 10)
        self.sub = self.create_subscription(
                Twist, '/jetbot_cmdvel',
                self.sim.on_twist_detection, 10)
        self.timer = self.create_timer(
                0.5,
                self.node_timer_callback)

    def node_timer_callback(self):
        msg = self.sim.timer_callback()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    arucosim = ARUCOSimulatorNode()

    rclpy.spin(arucosim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arucosim.destroy_node()
    rclpy.shutdown()

def no_ros_main():
    x_start = 20 * random()
    y_start = 20 * random()
    x_goal = 20 * random()
    y_goal = 20 * random()
    theta_goal = 2 * np.pi * random() - np.pi
    theta_start = np.arctan2(y_goal - y_start, x_goal - x_start
                             ) + (random()*np.pi/6)-np.pi/12
    start = np.array([x_start, y_start, theta_start])
    goal = np.array([x_goal, y_goal, theta_goal])
    arucosim = ARUCOSimulator(
            start = start,
            goal = goal,
            # Set one marker at the goal
            # and one marker between the goal and 
            marker_list={1: (start + goal)/2,
                         2: goal})
    controller = RandomController(
            2,
            minu=np.array([-100, -np.pi*10]),
            maxu=np.array([100, np.pi*10]))
    random_controller = False
    move2aruco = MoveToARUCO(logger=PrintLogger())
    while True:
        aruco_msg = arucosim.timer_callback()
        if random_controller:
            control = controller.control(arucosim.state, arucosim.goal)
            twist = new_twist(control[0], control[1])
        else:
            twist = move2aruco.on_aruco_detection(aruco_msg)
            twist = new_twist(twist.linear.x*(-10), twist.angular.z*1)
        arucosim.on_twist_detection(twist)


if __name__ == '__main__':
    test_rotation_matrix()
    no_ros_main()
