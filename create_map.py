# import os
# from datetime import datetime
# # import progressbar

# import numpy as np
# # import open3d as o3d
# import pcl
# import pymap3d as pm

# import pykitti

# from transforms3d import affines,euler

# basedir = '/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_28'
# date = '2011_09_28'
# drive = ['0001','0002']
        

# def do_transform(point, trans):
#     # print(point.shape)
#     out = np.zeros((4,))
#     intensity = point[3]
#     point[3] = 1
#     out = np.dot(trans,point)
#     out[3] = intensity
#     return out


# def convert_pcl(kitti, pos):
#     velo_path = os.path.join(kitti.data_path, 'velodyne_points')
#     velo_data_dir = os.path.join(velo_path, 'data')
#     velo_filenames = sorted(os.listdir(velo_data_dir))
#     with open(os.path.join(velo_path, 'timestamps.txt')) as f:
#         lines = f.readlines()
#         velo_datetimes = []
#         for line in lines:
#             if len(line) == 1:
#                 continue
#             dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
#             velo_datetimes.append(dt)

#     iterable = zip(velo_datetimes, velo_filenames, pos)
#     # map = np.empty((0,4))
#     map = pcl.PointCloud()
#     for dt, filename, p in iterable:
#         if dt is None:
#             continue
#         velo_filename = os.path.join(velo_data_dir, filename)
        
#         scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)
#         # print(p[:3].reshape(3,))
#         rotation = euler.euler2mat(p[0],p[1],p[2])
#         # rotation = np.identity(3)
#         # print(rotation.shape)
#         translation = p[:3].reshape(3,)
#         # print(translation)
#         transform = affines.compose(translation, rotation, np.ones((3,)))
#         print(transform)
#         for i in range(scan.shape[0]):
#             scan[i] = do_transform(scan[i], transform)
#         pcd_out = pcl.PointCloud()
#         pcd_out.from_array(scan[:,:3])
#         # pcd_out = pcd_in.transform(transform)
#         # pcl.transformPointCloud(pcd_in, pcd_out, transform)
#         pcd_transformed = pcl.PointCloud()
#         if map.size > 0:
#             icp = map.make_IterativeClosestPoint()
#             # pcl.transformPointCloud(pcd_out, pcd_transformed, icp.getFinalTransformation())
#             converged, transf, estimate, fitness = icp.icp(map, pcd_out)
#             map += estimate
#         else:
#             pcd_transformed = pcd_out
#             map += pcd_transformed
#         # map = np.concatenate((map,scan), axis = 0)
#         # depth = np.linalg.norm(scan, 2, axis=1)
#         # pitch = np.arcsin(scan[:, 2] / depth) # arcsin(z, depth)
#         # fov_down = -24.8 / 180.0 * np.pi
#         # fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
#         # proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
#         # proj_y *= 64  # in [0.0, H]
#         # proj_y = np.floor(proj_y)
#         # proj_y = np.minimum(64 - 1, proj_y)
#         # proj_y = np.maximum(0, proj_y).astype(np.int32)  # in [0,H-1]
#         # proj_y = proj_y.reshape(-1, 1)
#         # scan = np.concatenate((scan,proj_y), axis=1)
#         # scan = scan.tolist()
#         # for i in range(len(scan)):
#         #     scan[i][-1] = int(scan[i][-1])
#         # print(scan.shape)
#     return map



# def convert_gps_fix(kitti):
#     print("Converting gps fix data")
#     result = []
#     lat0,lon0,h0 = 0,0,0
#     initilized = False
#     for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
#         if not initilized:
#             initilized = True
#             lat0 = oxts.packet.lat
#             lon0 = oxts.packet.lon
#             h0 = oxts.packet.alt

#         (x, y, z) = pm.geodetic2enu(oxts.packet.lat, oxts.packet.lon, oxts.packet.alt, lat0, lon0, h0)
#         result.append(np.array([x, y, z, oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw]))
#         # print(result[-1])
#     return result



# kitti = pykitti.raw(basedir, date, drive[0], frames=None)

# pos = convert_gps_fix(kitti)

# map = convert_pcl(kitti, pos)

# # print(map.shape)

# # device = o3d.core.Device("CPU:0")
# # dtype = o3d.core.Dtype.Float32
# # pcd = o3d.t.geometry.PointCloud(device)

# # pcd.point["positions"] = o3d.core.Tensor(map[:,0:3], dtype, device)
# # pcd.point["intensities"] = o3d.core.Tensor([[i] for i in map[:,3]], dtype, device)

# # # voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)

# # o3d.t.io.write_point_cloud("/home/xinyuwang/adehome/kitti_to_ros2bag/map.pcd", pcd)

# # pcd = o3d.geometry.PointCloud()
# # pcd.points = o3d.utility.Vector3dVector(map[:,0:3])
# # voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.7)
# # o3d.io.write_point_cloud("/home/xinyuwang/adehome/kitti_to_ros2bag/map.pcd", voxel_down_pcd)

# pcl.io.savePCDFileASCII("/home/xinyuwang/adehome/kitti_to_ros2bag/map.pcd", map)