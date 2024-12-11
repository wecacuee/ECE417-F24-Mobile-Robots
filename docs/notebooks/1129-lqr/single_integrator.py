"""
Taken from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/Control/move_to_pose/move_to_pose.py

Move to specified pose

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""
import matplotlib.pyplot as plt
import numpy as np
from random import random
from functools import partial

try:
    from lqr import LQRController
except ImportError:
    print("lqr.py not found. Put it next to %s to run" % __file__)
    raise

class Angle:
    @staticmethod
    def wrap(theta):
        return ((theta + np.pi) % (2*np.pi)) - np.pi

    @staticmethod
    def iswrapped(theta):
        return (-np.pi <= theta) & (theta < np.pi)

    @staticmethod
    def diff(a, b):
        assert Angle.iswrapped(a).all()
        assert Angle.iswrapped(b).all()
        # np.where is like a conditional statement in numpy 
        # but it operates on per element level inside the numpy array
        return np.where(a < b,
                        (2*np.pi + a - b),
                        (a - b))
    @staticmethod
    def dist(a, b):
        # The distance between two angles is minimum of a - b and b - a.
        return np.minimum(Angle.diff(a, b), Angle.diff(b, a))

def single_integrator(state, control, dt, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED):
    control = np.sign(control) * np.minimum(np.abs(control), MAX_LINEAR_SPEED)

    next_state = state + control * dt
    return next_state

def move_to_pose(controller,
                 x_start, y_start, theta_start, x_goal, y_goal, theta_goal,
                 dt = 0.01,
                 system_dynamics = partial(
                     single_integrator,
                     # Robot specifications
                     MAX_LINEAR_SPEED = 15,
                     MAX_ANGULAR_SPEED = 7),
                 show_animation = True,
                 T = 100,
                ):

    pos_goal = np.array([x_goal, y_goal])
    x = np.array([x_start, y_start])
    theta = theta_start

    state = np.hstack((x, theta))
    x_diff = pos_goal - x

    pos_traj = []

    rho = np.linalg.norm(x_diff)
    for t in range(T):
        if not (rho > 0.001 and np.abs(Angle.diff(np.asarray(theta_goal),
                                                np.asarray(theta))) > 0.001):
            break
        pos_traj.append(x)

        u = controller.calc_control_command(
            t, x, pos_goal, theta, theta_goal)
        state = system_dynamics(state, u, dt)
        x = state[:2]
        theta = state[2]
        x_diff = pos_goal - x
        rho = np.linalg.norm(x_diff)

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.arrow(x_start, y_start, np.cos(theta_start),
                      np.sin(theta_start), color='r', width=0.1)
            plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=0.1)
            plot_vehicle(x[0], x[1], theta, 
                         [p[0] for p in pos_traj],
                         [p[1] for p in pos_traj],
                        dt=dt)


def plot_vehicle(x, y, theta, x_traj, y_traj, dt):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

def main():
    # simulation parameters
    dt = 0.01
    T = 100
    n = 3
    m = 3
    lqr_controller = LQRController(
            Qs = [np.zeros((n,n))]*T+[np.diag([1., 1., 1.])*T],
            Rs = [np.eye(m)*0.01]*T,
            As = [np.eye(n)]*T,
            Bs = [np.eye(m)*dt]*T,
            T  = T)
    controller = lqr_controller

    for i in range(5):
        x_start = 20 * random()
        y_start = 20 * random()
        theta_start = 2 * np.pi * random() - np.pi
        x_goal = 20 * random()
        y_goal = 20 * random()
        theta_goal = 2 * np.pi * random() - np.pi
        print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
              (x_start, y_start, theta_start))
        print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
              (x_goal, y_goal, theta_goal))
        move_to_pose(controller,
                     x_start, y_start, theta_start, x_goal, y_goal,
                     theta_goal,
                     T = T,
                     dt = dt)

if __name__ == '__main__':
    main()

