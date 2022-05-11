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

import pykitti


basedir = '/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_28'
date = '2011_09_28'
drive = ['0001','0002']

kitti = pykitti.raw(basedir, date, drive[0], frames=None)

for i in range(21):
    print(kitti.oxts[i].T_w_imu)
