# Python builtins
from enum import Enum

# Other packages
import numpy as np
import matplotlib.pyplot as plt

# ROS builtin packages
import rclpy
from rclpy.node import Node

# ROS other packages
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist

# My packages

# Get priority queue updateable from hw2
from .hw2_solution import PriorityQueueUpdatable
# Get astar2 from here:
# https://colab.research.google.com/github/wecacuee/ECE498-Mobile-Robots/blob/master/docs/notebooks/10-06-aruco-obstacle-avoid/aruco-obstacle-avoid.ipynb#scrollTo=af314add-33c4-472f-833d-fdc27c14fc75
from .astar2 import astar, backtrace_path

# Get ObstacleListToGraph, Robot and MapProperties from here:
# https://colab.research.google.com/github/wecacuee/ECE498-Mobile-Robots/blob/master/docs/notebooks/10-06-aruco-obstacle-avoid/aruco-obstacle-avoid.ipynb#scrollTo=af314add-33c4-472f-833d-fdc27c14fc75
from .obstacle_list_to_graph import ObstacleListToGraph, Robot, MapProperties, Angle

from .transform_utils import axangle_from_quat, rotmat

VMIN = 0.07
OMIN = np.pi # one rotation per two sec
DT = 0.2
GOAL_MARKER_ID = 7
PYPLOT_DEBUG = True

def plot_map(ax, map_properties, goal, current_state,
            goalsize=0.1,
            robotsize=0.1,
            obstaclecolor='r',
            goalcolor='g'):
    state_min = map_properties.state_min
    state_max = map_properties.state_max
    obstacles = map_properties.obstacles
    ax.set_xlim(state_min[0], state_max[0])
    ax.set_ylim(state_min[1], state_max[1])
    ax.axis('equal') # keeps square obstacles square
    
    # Draw the obstacle as rectangles
    # Had to google matplotlib + draw rectangle
    # https://stackoverflow.com/questions/37435369/how-to-draw-a-rectangle-on-image
    for obs in obstacles:
        xy = obs[:2]
        width, height = obs[2:]
        orect = patches.Rectangle(xy, width, height, facecolor=obstaclecolor)
        # Add the patch to the Axes
        ax.add_patch(orect)
    
    # Draw the goal as a rectangle
    grect = patches.Rectangle(goal, goalsize, goalsize, facecolor=goalcolor, label='goal')
    ax.add_patch(grect)
    
    # Draw the robot as an arrow
    # Google: matplotlib draw arrow
    ax.arrow(current_state[0], current_state[1], 
             robotsize*np.cos(current_state[2]), robotsize*np.sin(current_state[2]),
             width=0.08*robotsize,
             label='robot')
    return ax

def goal_check_region(m, goal, goal_wh=(0.1, 0.1)):
    m = np.asarray(m)
    goal = np.asarray(goal)
    goal_wh = np.asarray(goal_wh)
    mdiff = (m[:2] - goal)
    return ((0 <= mdiff) & (mdiff <= goal_wh)).all()

def euclidean_heurist_dist(node, goal):
    x_n, y_n, theta_n = node
    x_g, y_g = goal
    return np.sqrt((x_n-x_g)**2 + (y_n - y_g)**2)

class MoveStopCycle(Enum):
    MOVE = 0
    STOP = 1

def shortest_path_astar(state, obstacles, goal):
    map = MapProperties(
        state_min = np.array([
            -0.5, # x min in meters
            -0.0, # y min in meters
            -np.pi # theta min in radians
        ]),
        state_max = np.array([
            0.5, # x max in meters
            1.0, # y max in meters
            np.pi # theta max in radians
        ]),
        state_discrete_step = np.array([
            0.01, # x min in meters
            0.01, # y min in meters
            np.pi/20 # theta min in radians
        ])
        obstacles = np.array([(obs[0], obs[1], 0.10, 0.10)
                              for obs in obstacles])
    )
    robot = Robot(linvel_range = np.array([VMIN, VMIN]),
                  angvel_range = np.array([OMIN, OMIN]),
                  wheel_base = 0.12,
                  dt = DT)
    graph = ObstacleListToGraph(map, robot)
    start_state = state
    start_node_tuple = tuple(state.tolist())
    success, search_path, node2parent, node2dist = astar(
        graph,
        euclidean_heurist_dist,
        start_node_tuple, goal[:2],
        goal_check=goal_check_region)
    path = list(backtrace_path(node2parent, start_node_tuple, search_path[-1]))
    if os.environ.get('DISPLAY', '') and PYPLOT_DEBUG:
        fig, ax = plt.subplots()
        ax.quiver(path[:, 0], path[:, 1], np.cos(path[:, 2]), np.sin(path[:, 2]), 1., scale=80, label='path')
        plot_map(ax, map, goal, start_state)
        ax.legend()
        plt.pause(0.001)
    return path

def pose3D_to_pose2D(geometry_pose):
    axis, angle = axangle_from_quat(np.array(
            [pose.orientation.w,
             pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z]))
    if not np.allclose(axis, [0, 1, 0]):
        assert False, '''Please rotate the marker so that only one axis is rotated'''
    pose = np.array(
         [m.pose.position.x, m.pose.position.z,
          angle])
    return pose

def invert_pose2D(xytheta):
    """
    Returns the inverted pose
    """
    return np.hstack((
            - rotmat(-xytheta[2]) @ xytheta[:2],
            - xytheta[2]))

def pose_multiply(xytheta1, xytheta2):
    R1 = rotmat(xytheta1[2])
    R2 = rotmat(xytheta2[2])
    t1 = xytheta1[:2]
    t2 = xytheta1[:2]
    # R1 (R2 @ x + t2) + t1
    # R1 @ R2 @ x + R1 @ t2 + t1
    theta12 = Angle.wrap(xytheta1[2] + xytheta2[2])
    t12 = R1 @ t2 + t1
    res = np.zeros(3)
    res[2] = theta12
    res[:2] = t12
    return res

class RobotObstaclePosesFromMarkers():
    def __init__(self):
        self.prev_marker_poses_wrt_robot = dict()

    def _fill_in_missing_markers(self, marker_poses_wrt_robot):
        if not len(self.prev_marker_poses_wrt_robot):
            # if there are no previously recorded markers,
            # there cannot be any filling in.
            return marker_poses_wrt_robot

        # if there is any overlap between previous markers and current
        # markers, then we can convert all previous markers to current
        # that were not seen this time.

        # set intersection 
        common_mrkr_id = list(marker_poses_wrt_robot.keys() &
                         self.prev_marker_poses_wrt_robot.keys())
        # any of the common marker ids will do if they exist
        if not len(common_mrkr_id):
            # if there is no common marker, then there cannot be any fill in 
            return marker_poses_wrt_robot
        else:
            ref_mrkr_id = common_mrkr_id[0]
            # ref_mrkr_id must exist in both the dicts prev and current.
            # That's what set intersection means
            change_in_cur_mrkr_wrt_prev = pose_multiply(
                marker_poses_wrt_robot[ref_mrkr_id],
                invert_pose2D(self.prev_marker_poses_wrt_robot[ref_mrkr_id]))

            # Propagate the prev_markers to the current pose dict
            for mrkr_id, mrkr_pose in self.prev_marker_poses_wrt_goal.items():
                if mrkr_id not in marker_poses_wrt_robot:
                    marker_poses_wrt_robot = pose_multiply(
                        change_in_cur_mrkr_wrt_prev,
                        mrkr_pose)
        return marker_poses_wrt_robot

    def convert(self, msg):
        marker_poses_wrt_robot = dict()
        for marker in msg.markers:
            marker_poses_wrt_robot[marker.marker_id] = \
                    pose3D_to_pose2D(marker.pose)

        # Fill in the missing markers
        marker_poses_wrt_robot = \
                self._fill_in_missing_markers(marker_poses_wrt_robot)
        goal_pose_wrt_robot = marker_poses_wrt_robot.get(GOAL_MARKER_ID, None)
        robot_pose_wrt_goal = None
        obstacle_pose_wrt_goal = dict()
        # We cannot do much if we have not seen the goal yet
        # We should mark this as being lost
        if goal_pose_wrt_robot is not None:
            robot_pose_wrt_goal = invert_pose2D(goal_pose_wrt_robot)
            obstacle_pose_wrt_goal = dict()
            for mrkr_id, mrkr_pose in marker_poses_wrt_robot.items():
                if mrkr_id != GOAL_MARKER_ID:
                    obstacle_pose_wrt_goal[mrkr_id] = pose_multiply(markr_pose,
                                  robot_pose_wrt_goal)

        # Save the dictionary for next time
        self.prev_marker_poses_wrt_robot = marker_poses_wrt_robot
        return robot_pose_wrt_goal, obstacle_pose_wrt_goal


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
        self.timer = self.create_timer(DT, self.timer_callback)
        self.next_twist_to_pub = new_twist(0., 0.)
        self.robot_obstacle_poses =  RobotObstaclePosesFromMarkers()
        self.robot_behaviour_state = {
            # The robot is lost. Rotate in place until marker is found
            lost : False,
            # How much have we rotated since we were last lost
            rotated_since_lost : 0,

            # The robot published a move command or a stop command.
            # We are going to cycle between the two for consistency
            move_stop_cycle: MoveStopCycle.STOP,

            # Are we ready for next cmd
            ready_for_next_cmd : False
        }


    def on_aruco_detection(self, msg):
        robot_pose, obstacles = self.robot_obstacle_poses.convert(msg)
        goal = np.array([0., 0., 0.]) # goal is at origin
        if robot_pose is not None:
            # We are not lost anymore
            self.robot_behaviour_state['lost'] = False
            self.robot_behaviour_state['rotated_since_lost'] = 0

            # Are we there yet?
            if np.linalg.norm(robot_pose, goal) < 0.1:
                self.get_logger().info('We have arrived')
                self.next_twist_to_pub = new_twist(0., 0.) # Stop
                return # Do nothing

            if not self.robot_behaviour_state['ready_for_next_twist']:
                # Do nothing if the robot is not ready for the twist command
                return

            # We are ready for next twist command
            path = shortest_path_astar(robot_pose, obstacles, goal)
            if not len(path):
                self.get_logger().warning('Path length is zero')
                self.next_twist_to_pub = new_twist(0., 0.) # Stop
                return # Do nothing

            first_step = path[0]
            start, end = first_step
            linvel = np.linalg.norm(end[:2] - start[:2]) / DT
            end_m_start = Angle.diff(end[2], start[2])
            start_m_end = Angle.diff(start[2], end[2])
            ang_vel = (end_m_start 
                       if abs(end_m_start) < abs(start_m_end) 
                       else start_m_end) / DT

            self.next_twist_to_pub = new_twist(linvel, ang_vel)

    def _publish_move_stop(self, linvel, ang_vel):
        moved = False
        # Alternate between move and stop cycles
        if self.robot_behaviour_state['move_stop_cycle'] = MoveStopCycle.MOVE:
            self.pub.publish(new_twist(linvel, ang_vel))
            moved = True
            self.robot_behaviour_state['move_stop_cycle'] = MoveStopCycle.STOP
        elif self.robot_behaviour_state['move_stop_cycle'] = MoveStopCycle.STOP:
            self.pub.publish(new_twist(0., 0.))
            moved = False
            self.robot_behaviour_state['move_stop_cycle'] = MoveStopCycle.MOVE
        else:
            assert False, 'Only two possible values for move_stop_cycle'
        return moved


    def timer_callback(self):
        if self.robot_behaviour_state['lost']:
            if self.robot_behaviour_state['rotated_since_lost'] < 2*np.pi
                # if the robot is lost, it is not see 
                if self._publish_move_stop(new_twist(0., OMIN)):
                    self.robot_behaviour_state['robot_behaviour_state'] + OMIN*DT
            else:
                # we have rotated enough, we give up
                self._publish_move_stop(new_twist(0., 0.))
        else:
            # The robot is not lost, we know where to go
            moved = self._publish_move_stop(self.next_twist_to_pub)
            if moved:
                self.next_twist_to_pub = new_twist(0., 0.)
                self.robot_behaviour_state['ready_for_next_twist'] = True



def demo_matplotlib_map(args=None):
    rclpy.init(args=args)

    astar = Astar(demo_matplotlib_map=True)

    rclpy.spin(astar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    astar.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    astar = Astar()

    rclpy.spin(astar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    astar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
