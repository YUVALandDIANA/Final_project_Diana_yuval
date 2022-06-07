import rosbag
import os
from ros_numpy.image import image_to_numpy
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array, pointcloud2_to_array
import cv2
from scipy.io import savemat

file_name = '/home/user/Desktop/new_recordings_sync_27.03.22/GOOD/syncmood_CLI_2022-03-28-10-36-21.bag '  # path of bag file
directoryI = '/home/user/Desktop/new_recordings_sync_27.03.22/GOOD/images'  # path of saved images
directoryP = '/home/user/Desktop/new_recordings_sync_27.03.22/GOOD/pc'  # path of saved PC
image_number = 0
pointcloud_number = 0
bag = rosbag.Bag(file_name)

num_messages = 0
keys = dict()
for topic, message, timestamp in bag.read_messages():
    keys[topic] = 1
    num_messages += 1
    print(type(message))

    if topic == '/sensor_sync/velodyne_points':
        found_velodyne = True
        pointcloud_number = pointcloud_number + 1
        velodyne_points_raw = message
        velodyne_points = pointcloud2_to_xyz_array(message)
        os.chdir(directoryP)
        filenameP = str(pointcloud_number) + '.mat'
        savemat(filenameP, {'velodyne_points': velodyne_points})

    elif topic == '/sensor_sync/image_raw':
        found_img = True
        image_number = image_number + 1;
        image_raw = message

        img_bayer = image_to_numpy(message)
        img_rgb = cv2.cvtColor(img_bayer, cv2.COLOR_BAYER_RG2RGB, 0)
        os.chdir(directoryI)
        filenameI = str(image_number) + '.png'
        cv2.imwrite(filenameI, img_rgb)

    elif topic == '/Inertial_Labs/sensor_data':
        # you cannot convert the message to numpy because its conversion is not supported
        found_sensor_data = True
        sensor_data = message

    elif topic == '/Inertial_Labs/ins_data':
        # you cannot convert the message to numpy because its conversion is not supported
        found_sensor_data = True
        ins_data = message

    elif topic == '/Inertial_Labs/gps_data':
        # you cannot convert the message to numpy because its conversion is not supported
        found_sensor_data = True
        gps_data = message

stop = 1
if False:
    # printing in a loop over all the msgs in the bag file, that are related to the lidar points cloud.
    for topic, message, timestamp in bag.read_messages(topics=['/velodyne_points']):
        lidar_arr = rnp.point_cloud2.pointcloud2_to_xyz_array(message)  # ,remove_nans=True)
        time_date = datetime.fromtimestamp(timestamp.to_time())
        print(time_date)
        print(lidar_arr)
        print(lidar_arr.shape)
        print(type(lidar_arr))
