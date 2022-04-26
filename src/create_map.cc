#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/transforms.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/icp_nl.h>
#include <pcl/filters/approximate_voxel_grid.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
// #include <pcl/registration/ndt.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>


using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using Ptr = PointCloud::Ptr;

namespace geodetic
{
    const double semimajor_axis = 6378137.0;
    const double semiminor_axis = 6356752.31424518;
    const double pi = 3.14159265359;

    double radians(double degree) {
        return (degree * (pi / 180));
    };

    std::vector<double> geodetic2ecef(double lat, double lon, double alt) {
        lat = sin(lat);
        lon = radians(lon);
        double N = semimajor_axis * semimajor_axis / sqrt(semimajor_axis * semimajor_axis * cos(lat) * cos(lat) + semiminor_axis * semiminor_axis * sin(lat) * sin(lat));
        double x = (N + alt) * cos(lat) * cos(lon);
        double y = (N + alt) * cos(lat) * sin(lon);
        double z = (N * (semiminor_axis / semimajor_axis) * (semiminor_axis / semimajor_axis) + alt) * sin(lat);
        return {x,y,z};
    };

    std::vector<double> uvw2enu(double u, double v, double w, double lat0, double lon0){
        lat0 = radians(lat0);
        lon0 = radians(lon0);
        double t = cos(lon0) * u + sin(lon0) * v;
        double east = -sin(lon0) * u + cos(lon0) * v;
        double up = cos(lat0) * t + sin(lat0) * w;
        double north = -sin(lat0) * t + cos(lat0) * w;
        return {east, north, up};
    };

    std::vector<double> geodetic2enu(double lat, double lon, double h, double lat0, double lon0, double h0) {
        std::vector<double> result1 = geodetic2ecef(lat, lon, h);
        std::vector<double> result2 = geodetic2ecef(lat0, lon0, h0);
        return uvw2enu(result1[0] - result2[0], result1[1] - result2[1], result1[2] - result2[2], lat0, lon0);
    };

}

namespace pcd_mapping
{
    const int length = 10;
    struct Pose{
        double x;
        double y;
        double z;
        double lat;
        double lon;
        double alt;
        double roll;
        double pitch;
        double yaw;
        double linear_x;
        double linear_y;
        double linear_z;
        double angular_roll;
        double angular_pitch;
        double angular_yaw;
        double acc_x;
        double acc_y;
        double acc_z;
    };

    std::string pad_name(int index) {
        std::string idx = std::to_string(index);
        return std::string(length - idx.size(), '0') + idx;
    };

    Ptr read_cloud(std::string& input) {
        Ptr cloud(new PointCloud);
        std::fstream file(input.c_str(), std::ios::in | std::ios::binary);
	    if(!file.good()){
		    std::cerr << "Could not read file: " << input << std::endl;
		    return cloud;
	    }
	    file.seekg(0, std::ios::beg);
        for (int i=0; file.good() && !file.eof(); i++) {
            PointType point;
            file.read((char *) &point.x, 3*sizeof(float));
            file.read((char *) &point.intensity, sizeof(float));
            cloud->push_back(point);
	    }
	    file.close();
        std::cout << "Reading " + input + " Done !" << std::endl;
        return cloud;
    };

    std::vector<Pose> load_poses(std::string path_name) {
        std::vector<Pose> poses;
        int idx = 0;
        bool init = false;
        double lat0 = 0.0, lon0 = 0.0, h0 = 0.0;
        std::string file_name = path_name + "/" + pad_name(idx) + ".txt";
        // std::cout << "Path :" << file_name << std::endl;
        std::ifstream infile(file_name, std::ifstream::in);
        while(infile.good()) {
            std::string line;
            // std::cout << "Go" << std::endl;
            std::getline(infile, line);
            std::stringstream ss(line);
            Pose p;
            std::string tmp;
            ss >> tmp;
            p.lat = std::stod(tmp);
            ss >> tmp;
            p.lon = std::stod(tmp);
            ss >> tmp;
            p.alt = std::stod(tmp);
            ss >> tmp;
            p.roll = std::stod(tmp);
            ss >> tmp;
            p.pitch = std::stod(tmp);
            ss >> tmp;
            p.yaw = std::stod(tmp);
            ss >> tmp;
            p.linear_x = std::stod(tmp);
            ss >> tmp;
            p.linear_y = std::stod(tmp);
            ss >> tmp;
            p.linear_z = std::stod(tmp);
            if(!init) {
                init = true;
                lat0 = p.lat;
                lon0 = p.lon;
                h0 = p.alt;
            }

            std::vector<double> enu = geodetic::geodetic2enu(p.lat, p.lon, p.alt, lat0, lon0, h0);
            p.x = enu[0];
            p.y = enu[1];
            p.z = enu[2];

            poses.push_back(p);
            infile.close();
            idx++;
            file_name = path_name +  "/" + pad_name(idx) + ".txt";
            infile.open(file_name, std::ifstream::in);
        }
        // FILE *fp = fopen(file_name.c_str(),"r");
        // if (!fp)
        // return poses;
        // while (!feof(fp)) {
        //     Pose P;
        //     if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
        //         &P.lat, &P.lon, &P.alt, &P.roll,
        //         &P.pitch, &P.yaw, &P.linear_x, &P.linear_y,
        //         &P.linear_z, &P.val[2][1], &P.val[2][2], &P.val[2][3] )==12) {
        //         poses.push_back(P);
        //     }
        // }
        // fclose(fp);

        return poses;
    }
}

bool process_track(std::string pose_path, std::string pcd_path, Ptr map) {
    using pcd_mapping::Pose;
    std::vector<pcd_mapping::Pose> poses = pcd_mapping::load_poses(pose_path);

    for(int i = 0;i < poses.size(); i++) {
        std::string pcd_name = pcd_path + "/" + pcd_mapping::pad_name(i) + ".bin";
        Ptr cloud = pcd_mapping::read_cloud(pcd_name);
        Ptr filtered_cloud (new PointCloud);
        pcl::ApproximateVoxelGrid<PointType> filter;
        filter.setLeafSize (0.4, 0.4, 0.4);
        filter.setInputCloud (cloud);
        filter.filter (*filtered_cloud);
        pcd_mapping::Pose p = poses[i];
        Eigen::AngleAxisf yawAngle(p.roll, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf pitchAngle(p.yaw, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rollAngle(p.pitch, Eigen::Vector3f::UnitX());

        Eigen::Quaternion<float> q = rollAngle * pitchAngle * yawAngle;

        Eigen::Matrix3f rotation = q.matrix();
        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block<3,3>(0,0) = rotation;
        Eigen::Vector3f translation(p.x, p.y, p.z);
        trans.block<3,1>(0,3) = translation;
        
        std::cout << trans << std::endl;
        // Ptr trans_cloud(new PointCloud);
        // pcl::transformPointCloud(*cloud, *trans_cloud, trans);
        if(map->size() == 0) {
            Ptr trans_cloud(new PointCloud);
            pcl::transformPointCloud(*filtered_cloud, *trans_cloud, trans);
            *map += *trans_cloud;
        }
        else {
            // pcl::NormalDistributionsTransform<PointType, PointType> ndt;
            pclomp::NormalDistributionsTransform<PointType, PointType> ndt;
            ndt.setTransformationEpsilon (0.01);
            ndt.setStepSize (0.1);
            ndt.setResolution (1.0);
            ndt.setMaximumIterations (15);
            ndt.setInputSource (filtered_cloud);
            ndt.setInputTarget (map);
            ndt.setNumThreads(8);
            Ptr out_cloud(new PointCloud);
            ndt.align(*out_cloud, trans);
            pcl::transformPointCloud(*filtered_cloud, *out_cloud, ndt.getFinalTransformation());
            *map += *out_cloud;

        }

        std::cout << "Map size : " << map->size() << std::endl;

    }

    std::cout << "Pose size: " << poses.size() << std::endl;
}

int main() {
    std::string pose_path = "/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_28/2011_09_28/2011_09_28_drive_0001_sync/oxts/data";
    
    // for(auto& it : result) {
    //     std::cout << it.lat << " " << it.lon << " " << it.alt << " " << it.roll << " " << it.pitch << " " << it.yaw << std::endl;
    // }

    std::string pcd_path = "/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_28/2011_09_28/2011_09_28_drive_0001_sync/velodyne_points/data";

    Ptr map(new PointCloud);

    if(process_track(pose_path, pcd_path, map)) {
        std::cerr << pose_path << " Failed" << std::endl;
    }

    Ptr filtered_map (new PointCloud);
    pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.5, 0.5, 0.5);
    approximate_voxel_filter.setInputCloud (map);
    approximate_voxel_filter.filter (*filtered_map);
    pcl::io::savePCDFileASCII("/home/xinyuwang/adehome/kitti_to_ros2bag/map.pcd", *filtered_map);

    return 0;
}