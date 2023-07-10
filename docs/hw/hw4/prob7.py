import numpy as np
if __name__ == '__main__':
    p = np.array([[0, 100, -1, 0.0]]).T
    R = np.array([[1.0, 0, 0],
                  [0, 0, 1],
                  [0, -1, 0]])
    t = np.array([[0, 0, 1]]).T
    K = np.array([[100., 0., 100.],
                  [0, 100., 50.],
                  [0, 0, 1]])
    Kinv = np.linalg.inv(K)
    u = np.array([[100, 75, 1.]]).T

    num = (-p[:3, :].T @ t)
    den = (p[:3, :].T @ R @ Kinv @ u)
    𝜆 =  num / den
    print("𝜆 = ", 𝜆)
    X_w = 𝜆 * R @ Kinv @ u + t
    print("X_w = ", X_w)
