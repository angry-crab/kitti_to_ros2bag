import os
from datetime import datetime
# import progressbar

import numpy as np
# from rosbags.rosbag2 import Writer
# from rosbags.serde import serialize_cdr
# # from rosbags.typesys.types import std_msgs__msg__String as String
# from rosbags.typesys.types import builtin_interfaces__msg__Time as Time
# from rosbags.typesys.types import std_msgs__msg__Header as Header
# from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
# from rosbags.typesys.types import geometry_msgs__msg__Quaternion as Quaternion
# from rosbags.typesys.types import sensor_msgs__msg__PointField as PointField
# from rosbags.typesys.types import sensor_msgs__msg__PointCloud2 as Pcl2
# from rosbags.typesys.types import sensor_msgs__msg__Imu as Imu
# from rosbags.typesys.types import sensor_msgs__msg__NavSatFix as NavSatFix
# from rosbags.typesys.types import geometry_msgs__msg__TwistStamped as TwistStamped

import rosbag2_py

from rclpy.serialization import serialize_message
# from std_msgs.msg import String

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Twist, TwistStamped
from rclpy.time import Time

import pykitti

import tf_transformations

basedir = '/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_26'
date = '2011_09_26'
drive = '0001'

def save_imu(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    imu_topic_info = rosbag2_py._storage.TopicMetadata(
    name=topic,
    type='sensor_msgs/msg/Imu',
    serialization_format='cdr')
    bag.create_topic(imu_topic_info)
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        t = float(timestamp.strftime("%s.%f"))
        timer = Time(seconds = int(t), nanoseconds = int((t - int(t))* 10**6))
        head = Header(stamp=timer.to_msg(),frame_id=imu_frame_id)
        q = tf_transformations.quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        quaternion = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
        angular_velocity = Vector3(x = oxts.packet.wf, y = oxts.packet.wl, z = oxts.packet.wu)
        linear_acceleration = Vector3(x = oxts.packet.af, y = oxts.packet.al, z = oxts.packet.au)
        cov = np.zeros((9,))
        # cov[0,0] = cov[1,1] = cov[2,2] = 0.00001
        imu_msg = Imu(
            header = head,
            orientation = quaternion,
            orientation_covariance = cov,
            angular_velocity = angular_velocity,
            angular_velocity_covariance = cov,
            linear_acceleration = linear_acceleration,
            linear_acceleration_covariance = cov,
        )
        bag.write(topic, serialize_message(imu_msg), timer.nanoseconds)
    
def save_pcl(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_topic_info = rosbag2_py._storage.TopicMetadata(
    name=topic,
    type='sensor_msgs/msg/PointCloud2',
    serialization_format='cdr')
    bag.create_topic(velo_topic_info)
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir))
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes.append(dt)

    iterable = zip(velo_datetimes, velo_filenames)
    for dt, filename in iterable:
        if dt is None:
            continue
        velo_filename = os.path.join(velo_data_dir, filename)

        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)
        t = float(datetime.strftime(dt, "%s.%f"))
        # timer = Time(seconds = int(datetime.strftime(dt, "%s")), nanoseconds = int(datetime.strftime(dt, "%f"))*10e7)
        # header = Header(stamp=Time(seconds=int(t // 10**9),nanoseconds=int(t % 10**9)).to_msg(),frame_id=velo_frame_id)
        timer = Time(seconds = int(t), nanoseconds = int((t - int(t))* 10**6))
        header = Header(stamp=timer.to_msg(),frame_id=velo_frame_id)

        fields = [PointField(name = 'x', offset = 0, datatype = PointField.FLOAT32, count = 1),
                  PointField(name = 'y', offset = 4, datatype = PointField.FLOAT32, count = 1),
                  PointField(name = 'z', offset = 8, datatype = PointField.FLOAT32, count = 1),
                  PointField(name = 'intensity', offset = 12, datatype = PointField.FLOAT32, count = 1)]
        pcl_msg = point_cloud2.create_cloud(header, fields, scan)
        # pcl_msg = Pcl2(
        #     header = header,
        #     height = 1,
        #     width = scan.shape[0],
        #     fields = fields,
        #     is_bigendian = False,
        #     point_step = 16,
        #     row_step = scan.shape[0]*16,
        #     data = scan,
        #     is_dense = True,
        # )
        # bag.write(conn,t,serialize_cdr(pcl_msg,pcl_msg.__msgtype__))
        bag.write(topic, serialize_message(pcl_msg), timer.nanoseconds)

def save_gps_fix(bag, kitti, gps_frame_id, topic):
    print("Exporting gps fix data")
    gps_fix_topic_info = rosbag2_py._storage.TopicMetadata(
    name=topic,
    type='sensor_msgs/msg/NavSatFix',
    serialization_format='cdr')
    bag.create_topic(gps_fix_topic_info)
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        t = float(timestamp.strftime("%s.%f"))
        timer = Time(seconds = int(t), nanoseconds = int((t - int(t))* 10**6))
        # timer = Time(seconds = int(timestamp.strftime("%s")), nanoseconds = int(timestamp.strftime("%f"))*10e7)
        header = Header(stamp=timer.to_msg(),frame_id=gps_frame_id)
        navsatfix_msg = NavSatFix(
            header = header,
            status = NavSatStatus(status = 1, service = 1),
            latitude = oxts.packet.lat,
            longitude = oxts.packet.lon,
            altitude = oxts.packet.alt,
            position_covariance = np.zeros((9,)),
            position_covariance_type = 0,
        )
        bag.write(topic, serialize_message(navsatfix_msg), timer.nanoseconds)

def save_gps_vel(bag, kitti, gps_frame_id, topic):
    print("Exporting gps vel data")
    # conn = bag.add_connection(topic, TwistStamped.__msgtype__, 'cdr', '')
    gps_vel_topic_info = rosbag2_py._storage.TopicMetadata(
    name=topic,
    type='geometry_msgs/msg/TwistStamped',
    serialization_format='cdr')
    bag.create_topic(gps_vel_topic_info)
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        t = float(timestamp.strftime("%s.%f"))
        timer = Time(seconds = int(t), nanoseconds = int((t - int(t))* 10**6))
        # timer = Time(seconds = int(timestamp.strftime("%s")), nanoseconds = int(timestamp.strftime("%f"))*10e7)
        header = Header(stamp=timer.to_msg(),frame_id=gps_frame_id)
        # t = float(timestamp.strftime("%s.%f"))
        # stamp = Time(sec=int(t // 10**9),nanosec=int(t % 10**9))
        linear = Vector3(
            x = oxts.packet.vf,
            y = oxts.packet.vl,
            z = oxts.packet.vu)
        angular = Vector3(
            x = oxts.packet.wf,
            y = oxts.packet.wl,
            z = oxts.packet.wu)
        twist = Twist(
            linear = linear,
            angular = angular)
        twist_msg = TwistStamped(
            header = header,
            twist = twist,
        )
        bag.write(topic, serialize_message(twist_msg), timer.nanoseconds)


# The 'frames' argument is optional - default: None, which loads the whole dataset.
# Calibration, timestamps, and IMU data are read automatically. 
# Camera and velodyne data are available via properties that create generators
# when accessed, or through getter methods that provide random access.
kitti = pykitti.raw(basedir, date, drive, frames=None)

# dataset.calib:         Calibration data are accessible as a named tuple
# dataset.timestamps:    Timestamps are parsed into a list of datetime objects
# dataset.oxts:          List of OXTS packets and 6-dof poses as named tuples
# dataset.camN:          Returns a generator that loads individual images from camera N
# dataset.get_camN(idx): Returns the image from camera N at idx  
# dataset.gray:          Returns a generator that loads monochrome stereo pairs (cam0, cam1)
# dataset.get_gray(idx): Returns the monochrome stereo pair at idx  
# dataset.rgb:           Returns a generator that loads RGB stereo pairs (cam2, cam3)
# dataset.get_rgb(idx):  Returns the RGB stereo pair at idx  
# dataset.velo:          Returns a generator that loads velodyne scans as [x,y,z,reflectance]
# dataset.get_velo(idx): Returns the velodyne scan at idx 

# print(len(data.timestamps))

# print(len(data.oxts))

# print(len(data.velo))

# print(kitti.timestamps[0])

# print(kitti.oxts[0])

# print(kitti.get_velo(0))

# point_velo = np.array([0,0,0,1])
# point_cam0 = data.calib.T_cam0_velo.dot(point_velo)

# point_imu = np.array([0,0,0,1])
# point_w = [o.T_w_imu.dot(point_imu) for o in data.oxts]

# for cam0_image in data.cam0:
#     # do something
#     pass

# cam2_image, cam3_image = data.get_rgb(3)

# with Writer(basedir+"/ros2_bag") as writer:
#     imu_topic = '/imu'
#     pcl_topic = '/point_cloud'
#     # gnss_topic = '/gnss'
#     # can_bus_topic = '/can_bus'
#     imu_frame_id = 'imu_link'
#     imu_topic = '/imu'
#     gps_fix_topic = '/gps_fix'
#     gps_vel_topic = '/gps_vel'
#     velo_frame_id = 'point_cloud'
#     velo_topic = '/velodyne'
#     # save_imu(writer, kitti, imu_frame_id, imu_topic)
#     # save_gps_fix(writer, kitti, imu_frame_id, gps_fix_topic)
#     # save_gps_vel(writer, kitti, imu_frame_id, gps_vel_topic)
#     save_pcl(writer, kitti, velo_frame_id, velo_topic)
    
writer = rosbag2_py.SequentialWriter()

storage_options = rosbag2_py._storage.StorageOptions(
    uri=basedir+"/ros2_bag",
    storage_id='sqlite3')
converter_options = rosbag2_py._storage.ConverterOptions('', '')
writer.open(storage_options, converter_options)

velo_topic = '/velodyne'
velo_frame_id = 'point_cloud'
save_pcl(writer, kitti, velo_frame_id, velo_topic)

imu_topic = '/imu'
imu_frame_id = 'imu_link'
save_imu(writer, kitti, imu_frame_id, imu_topic)

gps_fix_topic = '/gps_fix'
gps_fix_frame_id = 'gnss'
save_gps_fix(writer, kitti, gps_fix_frame_id, gps_fix_topic)

gps_vel_topic = '/gps_vel'
gps_vel_frame_id = 'gnss'
save_gps_vel(writer, kitti, gps_vel_frame_id, gps_vel_topic)
# writer.close()