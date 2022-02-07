import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def get_camera_intrinsic_matrix():
    K = o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
    )
    K = K.intrinsic_matrix
    return K

def generate_pixel_coordinates(shape):
    depth_x = np.mgrid[:shape[0], :shape[1]]
    depth_x_f = depth_x.astype(np.float64) + 0.5
    return depth_x_f

def to_homogeneous(euclidean):
    shape = euclidean.shape
    return np.concatenate(
        (euclidean,
         np.ones((1, *shape[1:]), dtype=euclidean.dtype)),
        axis=0)

def visualize(point_cloud):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud.transpose())
    o3d.io.write_point_cloud("00000.ply", pcd)
    o3d.visualization.draw_geometries([pcd])

def camera_model_example():
    color = o3d.io.read_image("00000.jpg")
    depth = o3d.io.read_image("00000.png")
    plt.show()
    depth_np = np.asarray(depth)
    depth_np_f = depth_np.astype(np.float64)
    plt.imshow(depth_np_f)
    plt.savefig("00000-depth.png")
    plt.imshow(depth_np_f[::40, ::40])
    plt.savefig("00000-depth-lowres.png")
    K = get_camera_intrinsic_matrix()
    Kinv = np.linalg.inv(K)
    depth_x_f_h = to_homogeneous(generate_pixel_coordinates(depth_np_f.shape))
    point_cloud = Kinv @ depth_x_f_h.reshape(3, -1)
    point_cloud = point_cloud / point_cloud[2, :] * depth_np_f.reshape(1, -1)
    visualize(point_cloud)

if __name__ == '__main__':
    camera_model_example()
