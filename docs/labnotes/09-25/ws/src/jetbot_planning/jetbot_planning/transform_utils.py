import numpy as np
# https://github.com/cgohlke/transformations/blob/master/transformations/transformations.py
def quaternion_about_axis(angle, axis, _EPS=1e-6):
    """Return quaternion for rotation about axis.

    >>> q = quaternion_about_axis(0.123, [1, 0, 0])
    >>> numpy.allclose(q, [0.99810947, 0.06146124, 0, 0])
    True

    """
    axis = np.asarray(axis)
    q = np.zeros((*axis.shape[:-1], axis.shape[-1]+1))
    q[..., 1:] = axis
    qlen = np.linalg.norm(q, axis=-1)
    if qlen > _EPS:
        q *= np.sin(angle / 2.0) / qlen
    q[..., 0] = np.cos(angle / 2.0)
    return q

def axangle_from_quat(quat):
    """Return axis-angle for rotation from quaternion

    >>> q = quaternion_about_axis(0.123, [1, 0, 0])
    >>> axis, angle = axangle_from_quat(q)
    >>> numpy.allclose(axis, [1, 0, 0])
    True
    >>> numpy.allclose(angle, 0.123)
    True

    """
    quat /= np.linalg.norm(quat, axis=-1)
    sin_theta_by_2 = np.linalg.norm(quat[..., 1:4], axis=-1)
    cos_theta_by_2 = quat[..., 0]
    theta = 2 * np.arctan2(sin_theta_by_2, cos_theta_by_2)
    axis = quat[..., 1:5] / sin_theta_by_2
    return axis, theta

def rotmat(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

