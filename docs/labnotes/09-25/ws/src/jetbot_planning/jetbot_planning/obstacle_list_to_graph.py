from dataclasses import dataclass

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

@dataclass
class MapProperties:
    state_min : np.ndarray
    state_max : np.ndarray
    state_discrete_step : np.ndarray
    obstacles : np.ndarray


@dataclass
class Robot:
    linvel_range: np.ndarray
    angvel_range: np.ndarray
    dt: float
    wheel_base: float

def do_points_collide(map, xy_nbrs):
    # Nx2 with N neighbors and 2 dim x, y. the orientation of robot does not matter for obstacles
    in_bounds = (
        (map.state_min[:2] <= xy_nbrs) & 
        (xy_nbrs < map.state_max[:2])).all(axis=-1)
    
    obstacles_xy = map.obstacles[:, :2] # Ox2
    obstacles_wh = map.obstacles[:, 2:] # Ox2
    
    nbr_obs_diff = xy_nbrs[:, None, :] - obstacles_xy[None, :, :] # NxOx2
    is_nbr_in_obs = ((0 <= nbr_obs_diff) & (nbr_obs_diff < obstacles_wh)).all(axis=-1) # NxO
    is_nbr_in_any_obs = is_nbr_in_obs.any(axis=-1) # N
    return is_nbr_in_any_obs | (~in_bounds)


class ObstacleListToGraph:
    def __init__(self, map_properties, robot):
        self.map = map_properties
        self.robot = robot

    def _discretize(self, s):
        # divide by the step size, then round to convert the number to the nearest integer
        # and then multiply by the step size again.
        return np.round(s / self.map.state_discrete_step) * self.map.state_discrete_step
        
    def _get_bounds(self, current_state):
        vmin, vmax = self.robot.linvel_range
        omegamin, omegamax = self.robot.angvel_range
        step_size = self.map.state_discrete_step
        dt = self.robot.dt
        s = np.asarray(current_state)

        # The max and min state cells are governed by 
        # max and min velocity both linear and angular
        s = self._discretize(s)
        state_min = s + np.array([
            vmin * np.cos(s[2]) * dt,
            vmin * np.sin(s[2]) * dt,
            omegamin * dt])
        state_min = self._discretize(state_min)
        xy_min = state_min[:2]
        theta_min_angle = Angle.wrap(state_min[2]) # Angle type
        
        state_delta = np.array([
            (vmax - vmin) * np.cos(s[2]) * dt,
            (vmax - vmin) * np.sin(s[2]) * dt,
            (omegamax - omegamin) * dt
        ])
        state_delta = self._discretize(state_delta)
        xy_delta = state_delta[:2]
        theta_delta = state_delta[2]

        return s, xy_delta, xy_min, theta_delta, theta_min_angle, step_size
        
    def get_all_nbrs(self, current_state, default):
        s, xy_delta, xy_min, theta_delta, theta_min_angle, step_size = self._get_bounds(current_state)
        xy_uniq_nbrs = self.get_lin_vel_nbrs(s, xy_delta, xy_min, step_size)
        
        xy_uniq_nbrs_opp = self.get_lin_vel_nbrs(s, -xy_delta, 
                                                 s[:2] - (xy_min - s[:2]), # xy_min in opposite dir
                                                 step_size)
        xy_both_side_nbrs = np.vstack((xy_uniq_nbrs, 
                                       xy_uniq_nbrs_opp))
        
        theta_nbrs_angle = self.get_ang_vel_nbrs(s, theta_delta, theta_min_angle, step_size)
        theta_angle = s[2] # Angle type
        
        # Note that for theta_nbrs_angle and theta_angle we use diff, but once we have that, we
        # use normal substraction with theta_angle.
        theta_nbrs_opp_angle = Angle.wrap(theta_angle - Angle.diff(theta_nbrs_angle, theta_angle))
        theta_both_side_nbrs_angle = np.unique(np.hstack((theta_nbrs_angle,
                                               theta_nbrs_opp_angle)))
        
        state_nbrs = np.empty((len(xy_both_side_nbrs) + len(theta_both_side_nbrs_angle), 3))
        state_nbrs[:len(xy_both_side_nbrs), :2] = xy_both_side_nbrs 
        state_nbrs[:len(xy_both_side_nbrs), 2] = theta_angle
        
        state_nbrs[len(xy_both_side_nbrs):, :2] = s[:2]
        state_nbrs[len(xy_both_side_nbrs):, 2] = theta_both_side_nbrs_angle
        return state_nbrs
        
    def get_lin_vel_nbrs(self, s, xy_delta, xy_min, step_size):
        # all the cells that lie on the straight line between xy_delta+xy_min and xy_min are
        # possible neighbors.
        xy_dist = np.sqrt((xy_delta**2).sum())
        min_step = np.min(step_size[:2])
        xy_max_steps = xy_dist / min_step
        xy_dir = min_step * (xy_min - s[:2]) / np.linalg.norm((xy_min - s[:2]))

        xy_nbrs = np.arange(xy_max_steps+1)[:, None]*xy_dir + xy_min
        
        # Check neighbors for collision or being out of bound
        collisions = do_points_collide(self.map, xy_nbrs)
        if collisions[0]:
            return np.empty((0, 2)) # No nbr that does not collide
        indices, = np.nonzero(collisions)
        xy_non_colliding = xy_nbrs[:indices[0]-1] if len(indices) else xy_nbrs
        
        xy_uniq_nbrs = np.unique(
            np.round(xy_non_colliding / step_size[:2]).astype(dtype=np.int64),
            axis=0
        ) * step_size[:2]
        return xy_uniq_nbrs

    def get_ang_vel_nbrs(self, s, theta_delta, theta_min_angle, step_size):
        theta_steps = theta_delta / step_size[2]
        theta_nbrs_angle = np.unique(Angle.wrap(np.arange(theta_steps+1)*step_size[2] + theta_min_angle))
        return theta_nbrs_angle


    def get_nbrs_np(self, current_state, default):
        L = self.robot.wheel_base
        nbrs = self.get_all_nbrs(current_state, default)
        s = np.asarray(current_state)
        nbrs_diff = nbrs[:, :2] - s[:2]
        edge_cost = np.sqrt((nbrs_diff**2).sum(axis=-1)) + L*Angle.dist(nbrs[:, 2], s[2])/2
        return nbrs, edge_cost

    def get(self, current_state, default=[]):
        nbrs, edge_cost = self.get_nbrs_np(current_state, default)
        return [(tuple(nbr), ecost) 
                for nbr, ecost in zip(nbrs.tolist(), edge_cost.tolist())]
