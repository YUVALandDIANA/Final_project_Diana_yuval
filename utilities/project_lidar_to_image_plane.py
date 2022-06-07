from scipy.io import savemat
import scipy.io
import numpy as np

# 3D -> 3D velo to camera

# Transform matrix:
R = np.array([[-0.000830795488850, -0.036645123217335, 0.999327996567314],
              [-0.999822733583387, -0.018766802899451, -0.001519381050638],
              [0.018809869449297, -0.999152111569264, -0.036623035892467]])

t = np.array([[0.040999832514810], [-0.103757543960500], [-0.234292099334300]])


def transform_from_rot_trans(R, t):
    R = R.reshape(3, 3)
    t = t.reshape(3, 1)
    return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))


translation_mat = transform_from_rot_trans(R.T, t)

# PC load
mat = scipy.io.loadmat('/home/user/Desktop/Diana/pointcloud/1.mat')
velodyne_points = np.array(mat['velodyne_points'])  # For converting to a NumPy array
num_of_lines = len(velodyne_points)
ones = np.ones(num_of_lines, dtype=int)
lidar = velodyne_points.T
lidar = np.vstack((lidar, ones))

# 1st projection -> lidar to camera
lidar_points_to_camera_3d = np.matmul(translation_mat, lidar)

if False:
    lidar_points_to_camera_3d.shape

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(lidar_points_to_camera_3d[0, :], lidar_points_to_camera_3d[1, :], lidar_points_to_camera_3d[2, :])
    ax.scatter(lidar[0, :], lidar[1, :], lidar[2, :])
    plt.show()

# 3D -> 2D  camera to image

# Intrinsic matrix:
intrinsic = 1e3 * np.array(
    [[1.039641431856563, 0, 0], [0, 1.036702725659338, 0], [0.954962738197960, 0.601870249376522, 0.001000000000000]])

# 2nd projection -> camera to image
lidar_points_on_image_plane = np.matmul(intrinsic.T, (lidar_points_to_camera_3d[:3, :]))

# Filter point with negative z value
ind_inlier = np.where(lidar_points_on_image_plane[-1, :] >= 0)
lidar_points_on_image_plane = lidar_points_on_image_plane[:, ind_inlier[0]]

# Homogeneous Coordinates
lidar_points_on_image_plane_org = lidar_points_on_image_plane.copy()
lidar_points_on_image_plane[0, :] = lidar_points_on_image_plane[0, :] / lidar_points_on_image_plane[-1, :]
lidar_points_on_image_plane[1, :] = lidar_points_on_image_plane[1, :] / lidar_points_on_image_plane[-1, :]
lidar_points_on_image_plane[-1, :] = lidar_points_on_image_plane[-1, :] / lidar_points_on_image_plane[-1, :]

if False:
    ## project on image :
    from matplotlib import image, pyplot as plt
    import cv2

    image = cv2.imread('/home/user/Desktop/Diana/images/1.png')
    type(image)
    image.shape
    plt.figure()
    plt.scatter(lidar_points_on_image_plane[0, :], lidar_points_on_image_plane[1, :], marker=".", color="red", s=5)
    plt.imshow(image)
    plt.show()

if False:
    ## TEST find z value:

    xs = 750
    ys = 600
    ind = np.where(np.logical_and(
        np.logical_and(lidar_points_on_image_plane[0, :] > (xs - 10), lidar_points_on_image_plane[0, :] < (xs + 10)),
        np.logical_and(lidar_points_on_image_plane[1, :] > (ys - 50), lidar_points_on_image_plane[1, :] < (ys + 50))))
    np.median(lidar_points_on_image_plane_org[2, ind[0]])

    xs = 124
    ys = 700
    ind = np.where(np.logical_and(
        np.logical_and(lidar_points_on_image_plane[0, :] > (xs - 10), lidar_points_on_image_plane[0, :] < (xs + 10)),
        np.logical_and(lidar_points_on_image_plane[1, :] > (ys - 50), lidar_points_on_image_plane[1, :] < (ys + 50))))
    print(np.median(lidar_points_on_image_plane_org[2, ind[0]]))

    xs = 1805
    ys = 657
    ind = np.where(np.logical_and(
        np.logical_and(lidar_points_on_image_plane[0, :] > (xs - 10), lidar_points_on_image_plane[0, :] < (xs + 10)),
        np.logical_and(lidar_points_on_image_plane[1, :] > (ys - 50), lidar_points_on_image_plane[1, :] < (ys + 50))))
    print(np.median(lidar_points_on_image_plane_org[2, ind[0]]))
