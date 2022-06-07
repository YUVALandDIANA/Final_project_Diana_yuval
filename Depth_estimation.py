import scipy.io
import numpy as np


class target_in_frame:
    def __init__(self, track_id, class_type, xmin, ymin, xmax, ymax):
        # self.frame_num = frame_num
        self.ID = track_id
        self.class_type = class_type
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.xs = []
        self.ys = []
        self.z_depth_list = []
        self.dynamic = False


# 3D -> 3D velo to camera
# Transform matrix:
R_matrix = np.array([[-0.000830795488850, -0.036645123217335, 0.999327996567314],
                     [-0.999822733583387, -0.018766802899451, -0.001519381050638],
                     [0.018809869449297, -0.999152111569264, -0.036623035892467]])

t_matrix = np.array([[0.040999832514810], [-0.103757543960500], [-0.234292099334300]])


def transform_from_rot_trans(R_matrix, t_matrix):
    R_matrix = R_matrix.reshape(3, 3)
    t_matrix = t_matrix.reshape(3, 1)
    return np.vstack((np.hstack([R_matrix, t_matrix]), [0, 0, 0, 1]))


def target_definition(track_id, class_type, bbox):
    object_found = target_in_frame(track_id, class_type, int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
    return object_found


def coordinate_update(object, bbox):
    object.xmin = int(bbox[0])
    object.ymin = int(bbox[1])
    object.xmax = int(bbox[2])
    object.ymax = int(bbox[3])
    return object


def find_depth_of_object(object, pc_dir):
    translation_mat = transform_from_rot_trans(R_matrix.T, t_matrix)
    mat = scipy.io.loadmat(pc_dir)
    velodyne_points = np.array(mat['velodyne_points'])  # For converting to a NumPy array
    num_of_lines = len(velodyne_points)
    ones = np.ones(num_of_lines, dtype=int)
    lidar = velodyne_points.T
    lidar = np.vstack((lidar, ones))
    # 1st projection -> lidar to camera
    # from here we want new X and Y
    lidar_points_to_camera_3d = np.matmul(translation_mat, lidar)
    # 3D -> 2D  camera to image

    # Intrinsic matrix:
    intrinsic = 1e3 * np.array([[1.039641431856563, 0, 0], [0, 1.036702725659338, 0],
                                [0.954962738197960, 0.601870249376522, 0.001000000000000]])

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

    d_x = (object.xmax - object.xmin) / 2
    d_y = (object.ymax - object.ymin) / 2
    xs = (d_x + object.xmin)
    ys = (d_y + object.ymin)
    object.xs.append(xs)
    object.ys.append(ys)
    # calc the median of the list
    ind = np.where(np.logical_and(
        np.logical_and(lidar_points_on_image_plane[0, :] > (xs - d_x), lidar_points_on_image_plane[0, :] < (xs + d_x)),
        np.logical_and(lidar_points_on_image_plane[1, :] > (ys - d_y), lidar_points_on_image_plane[1, :] < (ys + d_y))))
    z_estimation = np.median(lidar_points_on_image_plane_org[2, ind[0]])
    object.z_depth_list.append(z_estimation)

    return object
