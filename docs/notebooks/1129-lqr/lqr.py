import numpy as np

def dare_backpropagation(Qs, Rs, As, Bs):
    # Dicrete algebraic Riccati equation
    P_T = Qs[-1]
    Ps = [P_T]
    # import pdb; pdb.set_trace()
    Ks = []
    for Q, R, A, B in reversed(list(zip(Qs[:-1], Rs, As, Bs))):
        K_T = np.linalg.solve(R + B.T @ P_T @ B,  B.T @ P_T @ A)
        P_T = A.T @ P_T @ A - A.T @ P_T @ B @ K_T + Q
        Ps.append(P_T)
        Ks.append(K_T)

    return list(reversed(Ps)), list(reversed(Ks))

class RandomController:
    def __init__(self, m, minu, maxu):
        self.m = m 
        self.minu = minu
        self.maxu = maxu
    def control(self, state, state_goal):
        return np.random.rand(self.m) * (self.maxu - self.minu) + self.minu

class LQRController:
    """
    Constructs an instantiate of the PIDController for navigating a
    3-DOF wheeled robot on a 2D plane
    """

    def __init__(self, Qs, Rs, As, Bs, N=10, T=10,
                 init_controller=None):
        self.Qs = Qs
        self.Rs = Rs
        self.As = As
        self.Bs = Bs
        self.N = N
        self.T = T
        Ps, Ks = dare_backpropagation(Qs, Rs, As, Bs)
        assert len(Ks) == T
        assert len(Ps) == T+1
        self.Ks = Ks
        self.Ps = Ps

    def calc_control_command(self, t, x, x_goal, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal

        Parameters
        ----------
        x : The current position in 2D
        x_goal : The target position in 2D

        Returns
        -------
        v : Command linear velocity in 2D
        """
        state_goal = np.hstack((x_goal, theta_goal))
        state = np.hstack((x, theta))
        return self.control(t, state, state_goal)

    def control(self, t, state, state_goal):
        # make goal the origin
        return -self.Ks[t] @ (state - state_goal)


