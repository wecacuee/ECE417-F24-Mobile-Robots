import time
import subprocess
import numpy as np
import matplotlib.mlab as mlab

def log(*args):
    log_message = ''.join([str(arg) for arg in args])
    print time.strftime('%H:%M:%S > ') + str(log_message)

def rowsort(rows):
    '''
    >>> X = np.arange(30).reshape(10,3)
    >>> S = X.copy()
    >>> np.random.shuffle(X)
    >>> np.allclose(S, rowsort(X))
    True
    '''
    sorted_inds = np.lexsort(rows.T)
    rows = rows[sorted_inds]
    return rows

def sort_inds_vals(inds, vals):
    sorted_inds = np.lexsort(inds.T)
    inds = inds[sorted_inds]
    vals = vals[sorted_inds]
    return inds, vals

def allclose(A_inds, A_vals, B_inds, B_vals):
    A_inds, A_vals = sort_inds_vals(A_inds, A_vals)
    B_inds, B_vals = sort_inds_vals(B_inds, B_vals)
    return np.allclose(A_vals, B_vals)

def add_pose_error(pose,dist,x_scale=0.5,rot_scale=10,seed=None):
    np.random.seed(seed)
    if dist == 'normal':
        dx =  np.random.normal(loc=0, scale=x_scale, size=3)
        drot =  np.random.normal(loc=0, scale=np.radians(rot_scale), size=3)
    elif dist == 'uniform':
        dx =  np.random.uniform(low=-x_scale, high=x_scale, size=3)
        drot =  np.radians(np.random.uniform(low=-rot_scale, high=rot_scale, size=3))
    else:
        print 'Unsupported distribution: ' + dist
        1/0
    
    err = np.hstack((dx, drot))
    print 'drot (rad):', drot
    print 'dx (m):', dx
    P = pose.get()
    pose.set(P+err)
    print 'Perturbed pose: ' + str(P + err)

def savewindow(window_name, file_name):
    '''Save the VPython window as an image to file.'''
    print 'Saving window', window_name, 'to', file_name
    # TODO this is very slow around 1 second try and find a better way
    time.sleep(2)
    subprocess.call(['import', '-window', window_name, file_name])
    # stop updates to the window interfering with the save?
    time.sleep(1)
    print 'Done'

def depth2xyzs(depthnp, K, color=None, depth_filter=None):
    rows, cols = depthnp.shape
    xres, yres = cols, rows
    yinds, xinds = np.indices((yres, xres))
    Kinv = np.linalg.inv(K)

    pts_hom = np.vstack((xinds.reshape(1, -1),
                         yinds.reshape(1, -1),
                         np.ones((1, len(yinds.ravel())))))

    depthnp = depthnp.ravel()
    if depth_filter is not None:
        depth_filter = depth_filter.ravel()
        depthnp = depthnp[depth_filter]
        pts_hom = pts_hom[:, depth_filter]

    pts3d = np.dot(Kinv, pts_hom)
    pts3d = pts3d / pts3d[2, :] * depthnp
    pts3d = pts3d.T
    if color is not None:
        rgbs = color.reshape(-1, 3)
        if depth_filter is not None:
            rgbs = rgbs[depth_filter, :]
        pts3d = np.hstack((pts3d, rgbs))
    return pts3d

def apply_transform(xyzs, T):
    assert len(xyzs.shape) == 2
    xyzs = T[:3, :3].dot(xyzs.T) + T[:3, 3].reshape(3, -1)
    return xyzs.T

def extrapolate_hemisphere_to_sphere(unit_normals):
    # Extrapolate hemisphere to a sphere
    # Convert to spherical coordinates and multiply theta by 2
    # theta = np.arccos(unit_normals[:, 2]/r)
    r = 1 # unit normals
    cos_theta = unit_normals[:, 2]/r
    cos_2_theta = 2*cos_theta**2 - 1
    condition = np.abs(cos_2_theta) <= 1
    assert np.all(condition), "cos theta > 1" + str(cos_2_theta[~condition])
    sin_2_theta = 2*cos_theta*np.sqrt(1 - cos_theta**2)
    assert np.all(np.abs(sin_2_theta) <= 1), "cos theta > 1"
    xsq_ysq = np.sqrt(unit_normals[:, 1]**2 + unit_normals[:, 0]**2)
    xsq_ysq[xsq_ysq == 0] = 1
    cos_phi = unit_normals[:, 0]/xsq_ysq
    sin_phi = unit_normals[:, 1]/xsq_ysq
    extrap_norms = np.empty_like(unit_normals)
    extrap_norms[:, 0] = sin_2_theta * cos_phi
    extrap_norms[:, 1] = sin_2_theta * sin_phi
    extrap_norms[:, 2] = cos_2_theta
    return extrap_norms

def rgbs_by_normals(normals, alpha=0):
    assert np.all(np.isreal(normals)), "normals not real"
    magnitudes = mlab.vector_lengths(normals, axis=1).reshape((-1, 1))
    magnitudes[magnitudes == 0] = 1
    unit_normals = normals / magnitudes

    # ## Coloring by hue space. Hue space is nice because it is cyclically
    # # continuous, but my conversion from hsv to rgb is bad because it is not
    # # continuous. FIX HSV to RGB with a continuous version. However, we need a
    # # hemispherical colorspace here, not cylinderical.
    # hue_normal = [0, 0, 1]
    # # This clips normals to hemisphere in 3D around the hue_normal, but
    # # creates symmetrical contours around the normal, which is not what we
    # # want. However, to do any better we would need a spherical color space.
    # # dot product is from [-1, 1], scale and shift it to [0, 1]
    # h1 = np.dot(unit_normals, hue_normal) * 0.5 + 0.5
    # assert np.all((h1 >= 0) & (h1 <= 1.0))
    # # calling the above value hue is good because we have continuous circular
    # # space for hue, which avoids discontinuities.
    # #sat_normal = [0, 1, 0]
    # #s = np.dot(unit_normals, sat_normal) * 0.5 + 0.5
    # s = np.ones_like(h1) * 0.3
    # v = np.ones_like(h1) * 0.7
    # rgb = colorsys_hsv_to_rgb(h1, s, v)
    # colors = (rgb * 255).astype(np.uint8)
    
    # # To get transparency effects
    # colors = np.empty((unit_normals.shape[0], 4), np.uint8)
    # colors[3] = alpha

    # # Simple absolute normal strategy
    # colors = np.abs(unit_normals) * 255

    # flip the normals. Choose the flipping plane carefully.
    flipping_plane = [0, 0, 1]
    unit_normals[np.dot(unit_normals, flipping_plane) < 0] *= -1
    extrap_norms = extrapolate_hemisphere_to_sphere(unit_normals)
    colors = extrap_norms * 127 + 128
    assert np.all(np.isreal(colors)), "Colors is not real"
    return np.asarray(colors, dtype=np.uint8)
