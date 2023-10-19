import numpy as np
from numpy.random import rand


def random_rotation():
    # Page 117-119 of Graphic Gems III by David Kirk, 1992
    # https://archive.org/details/isbn_9780124096738/page/117/mode/2up
    # "To generate uniformly distributed random rotations of a unit sphere,
    # first perform a random rotation about the vertical axis, then rotate the
    # north pole to a random position."
    # 
    # Construct the rotation matrix around the Z-axis
    # "Here we are assuming that the z-axis is the vertical axis, so the
    # "northpole" will be the point z = (0, 0, 1). 
    x1 = rand();
    theta = 2*np.pi*x1; # Pick a rotation about the pole (z-axis)
    Rtheta = np.array([[ np.cos(theta), np.sin(theta), 0],
                       [ -np.sin(theta), np.cos(theta), 0],
                       [ 0, 0, 1]]);
    # 
    # "Observe that we can take the point z to any other point p on the sphere
    # via a reflection through the plane orthogonal to the linze \bar{zp} and
    # containing origin. Such a reflection is given by the Householder matrix"
    # Rotate the Z-axis to a random orientation
    phi = 2*np.pi*rand(); # Pick a direction to deflect the pole
    z = rand(); # Pick the amount of pole deflection
    v_T = np.array([[np.cos(phi)*np.sqrt(z),
                     np.sin(phi)*np.sqrt(z),
                     np.sqrt(1-z)]])
    VV = v_T.T @ v_T
    # Householder matrix
    H = np.eye(3) - 2 * VV;
    # "To turn this into a rotation we need only apply one more reflection
    # (making the determinant positive). A convenient reflection for this
    # purpose si reflection through the origin---that is, scaling by -1."
    return - H @ Rtheta;

def is_valid_rot(R):
    # TODO: Put a valid rotation check here.
    # return true if rotation is a valid rotation
    return 0;


def angle_to_rot2D(theta):
    # Return the 2D rotation matrix corresponding to the angle theta
    # TODO: Replace with correct formula
    return np.array([[1, 0 ],
                     [0, 1]])

##################### Conversion from roll, pitch, yaw to Rotation matrix ###

def euler_to_rotmat(roll, pitch, yaw):
    # Return the 3D rotation matrix corresponding to the roll, pitch, yaw 
    Rx = np.eye(3)
    Ry = np.eye(3)
    Rz = np.eye(3)

    Rx[:2, :2] = angle_to_rot2D(roll); # this is correct
    # TODO: Construct Ry and Rz similar to Rx
    Ry[:2, :2] = angle_to_rot2D(pitch); # this is wrong
    Rz[:2, :2] = angle_to_rot2D(yaw); # this is wrong
    return Rz @ Ry @ Rx;

def rotmat_to_euler_rpy(R):
    roll= 0; # TODO: Replace with correct formula
    pitch = 0; # TODO: Replace with correct formula
    yaw = 0; # TODO: Replace with correct formula
    return (roll, pitch, yaw);

def test_euler_to_rot_and_back():
    print("\nnChecking roll")
    rpy = ( np.pi / 6, 0, 0)
    print("RPY:", rpy)
    Rrecovered = euler_to_rotmat(*rpy)
    print("Rrecovered:", Rrecovered)
    rpy_rec = rotmat_to_euler_rpy(Rrecovered)
    print("rpy_rec:" , rpy_rec)
    print("Is same?: " , (np.allclose(rpy_rec, rpy)))
    assert (np.allclose(rpy_rec, rpy))

    print("\nnChecking pitch")
    rpy = ( 0, np.pi/6, 0)
    print("RPY:", rpy)
    Rrecovered = euler_to_rotmat(*rpy)
    print("Rrecovered:", Rrecovered)
    rpy_rec = rotmat_to_euler_rpy(Rrecovered)
    print("rpy_rec:" , rpy_rec)
    print("Is same?: " , (np.allclose(rpy_rec, rpy)))
    assert (np.allclose(rpy_rec, rpy))

    print("\nnChecking yaw")
    rpy = ( 0, 0, np.pi/6)
    print("RPY:", rpy)
    Rrecovered = euler_to_rotmat(*rpy)
    print("Rrecovered:", Rrecovered)
    rpy_rec = rotmat_to_euler_rpy(Rrecovered)
    print("rpy_rec:" , rpy_rec)
    print("Is same?: " , (np.allclose(rpy_rec, rpy)))
    assert (np.allclose(rpy_rec, rpy))

    print("\nnChecking random rotation 10 times")
    for i in range(10):
        Rrand = random_rotation();
        print("Rrand:" , Rrand)
        print("is_valid:" , is_valid_rot(Rrand))
        rpy = rotmat_to_euler_rpy(Rrand)
        print("RPY:" , rpy )
        Rrecovered = euler_to_rotmat(*rpy)
        print("Rrecovered:" , Rrecovered )
        rpy_rec = rotmat_to_euler_rpy(Rrecovered)
        print("rpy_rec:" , rpy_rec)
        print("Is same?: " , (np.allclose(rpy_rec, rpy, atol=1e-5)))
        assert (np.allclose(rpy_rec, rpy, atol=1e-5))


##################### End: Conversion from roll, pitch, yaw to Rotation matrix ###

##################### Start: Conversion from axis-angle to Rotation matrix ###
def axangle_to_mat(axis, angle):
    # Returns the rotation matrix corresponding to the axis angle
    # representation
    R = np.eye(3)
    return R


def mat_to_axangle(R):
    # Returns the axis angle corresponding to the rotation matrix R
    # representation
    angle = 0
    axis = np.array([1., 0, 0])
    return axis, angle


def test_axangle_to_mat_and_back():
    print("\nnChecking random rotation 10 times")
    for i in range(10):
        Rrand = random_rotation()
        print("Rrand:" , Rrand)
        print("is_valid:" , is_valid_rot(Rrand))
        axis, angle = mat_to_axangle(Rrand)
        print("Axis-angle:" , axis, angle )
        Rrecovered = axangle_to_mat(axis, angle)
        print("Rrecovered:" , Rrecovered )
        axis_rec, angle_rec = axangle_to_mat(Rrecovered)
        print("AxAngle_rec:" , axis_rec, angle_rec)
        print("Angle: Is same?: " , (np.allclose(angle, angle_rec, atol=1e-5)))
        assert (np.allclose(angle, angle_rec, atol=1e-5))
        print("Axis: Is same?: " , (np.allclose(axis, axis_rec, atol=1e-5)))
        assert (np.allclose(axis, axis_rec, atol=1e-5))

##################### End: Conversion from axis-angle to Rotation matrix ###

##################### Start: Conversion from quaternion to axis-angle ###

def quat_to_axangle(quat):
    # Returns the axis angle corresponding to the quaternion
    # representation
    sintheta_by_2 = 0 # TODO: Replace by correct expression 
    costheta_by_2 = 0 # TODO: Replace by correct expression 
    angle = 0 # TODO: Replace by correct expression 
    axis = None # TODO: Replace by correct expression
    return axis, angle

def axangle_to_quat(axis, angle):
    # Returns the quaternion corresponding to the axis-angle
    # representation
    costheta_by_2  = 0 # TODO: Replace by correct expression
    quat = np.array([1., 0, 0, 0])
    quat[0] = 0 # TODO: Replace by correct expression
    quat[1:] = 0 # TODO: Replace by correct expression
    return quat

def quat_to_mat(quat):
    # Returns the rotation matrix corresponding to the quaternion
    # representation
    return axangle_to_mat(*quat_to_axangle(quat))


def mat_to_quat(R):
    # Returns the quaternion corresponding to the rotation matrix R
    # representation
    return axangle_to_quat(*mat_to_axangle(R))



def test_quat_to_mat_and_back():
    print("\nnChecking random rotation 10 times")
    for i in range(10):
        Rrand = random_rotation()
        print("Rrand:" , Rrand)
        print("is_valid:" , is_valid_rot(Rrand))
        quat = mat_to_quat(Rrand)
        print("Quat:" , quat)
        Rrecovered = quat_to_mat(quat)
        print("Rrecovered:" , Rrecovered )
        quat_rec = mat_to_quat(Rrecovered)
        print("quat_rec:" , quat_rec)
        print("Is same?: " , (np.allclose(quat_rec, quat, atol=1e-5)))
        assert (np.allclose(quat_rec, quat_rec, atol=1e-5))
##################### End: Conversion from quaternion to axis-angle ###

def transforamtion_matrix_from_R(R, trans):
    # Return 4x4 transformation matrix from 3x3 rotation matrix R and
    # 3x1 translation trans
    T = np.eye(4)
    return T

def transformation_matrix_from_rpy(roll, pitch, yaw, trans):
    R = euler_to_rotmat(roll, pitch, yaw)
    return # return the 4x4 transformation matrix



def check_gimbal_lock():
    print("\nnChecking gimbal lock")
    rpy = ( np.pi/3, np.pi/2, np.pi/6)
    print("RPY:", rpy)
    Rrecovered = euler_to_rotmat(*rpy)
    print("Rrecovered:", Rrecovered)
    rpy_rec = rotmat_to_euler_rpy(Rrecovered)
    print("rpy_rec:" , rpy_rec)
    print("Is same?: " , (np.allclose(rpy_rec, rpy)))

if __name__ == '__main__':
    test_euler_to_rot_and_back()
    check_gimbal_lock()
    test_axangle_to_mat_and_back()
    test_quat_to_mat_and_back()

