# kitti_to_ros2bag

```
sudo apt update
sudo apt install ros-galactic-tf-transformations ros-galactic-sensor-msgs-py

pip3 install transforms3d pykitti opencv-python
```

`create_bag.py` can be used to generate a ros2 bag using KITTI dataset. Set `base_dir`, `date`, and `drive` accordingly. 

`pcd_mapping` can be used to generate a pcd map. Set `pose_paths`, `pcd_paths`, and path to save the map accrodingly. 