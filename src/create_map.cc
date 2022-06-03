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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/ndt.h>
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

    std::vector<double> se3_translation(double lat, double lon, double h, double scale) {
        double tx,ty,tz;
        tx = scale * lon * pi * semimajor_axis / 180.0;
        ty = scale * semimajor_axis * log(tan((90.0 + lat) * pi / 360.0));
        tz = h;
        return {tx,ty,tz};
    };

    Eigen::Matrix3f rotx(double x) {
        Eigen::Matrix3f rot;
        rot.setIdentity();
        double c = cos(x), s = sin(x);
        rot(1,1) = c;
        rot(2,2) = c;
        rot(1,2) = -s;
        rot(2,1) = s;
        return rot;
    };

    Eigen::Matrix3f roty(double x) {
        Eigen::Matrix3f rot;
        rot.setIdentity();
        double c = cos(x), s = sin(x);
        rot(0,0) = c;
        rot(2,2) = c;
        rot(0,2) = s;
        rot(2,0) = -s;
        return rot;
    };

    Eigen::Matrix3f rotz(double x) {
        Eigen::Matrix3f rot;
        rot.setIdentity();
        double c = cos(x), s = sin(x);
        rot(0,0) = c;
        rot(1,1) = c;
        rot(0,1) = -s;
        rot(1,0) = s;
        return rot;
    };

}

namespace pcd_mapping
{
    const int length = 10;
    static bool init = false;
    static double lat0 = 0.0, lon0 = 0.0, h0 = 0.0, scale = 0.0, x0 = 0.0, y0 =0.0, z0 = 0.0;
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
        if(init) {
            // std::cout <<  "scale: " << scale << " x0: " << x0 << " y0: " << y0 << " z0: " << z0 << std::endl;
        }
        std::vector<Pose> poses;
        int idx = 0;
        std::string file_name = path_name + "/" + pad_name(idx) + ".txt";
        // std::cout << "Path :" << file_name << std::endl;
        std::ifstream infile(file_name, std::ifstream::in);
        while(infile.good()) {
            std::string line;
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

            // std::cout << std::setprecision(10) << p.lat << " " << p.lon << " " << p.alt << " " << p.roll << " " << p.pitch << " " << p.yaw << std::endl;

            if(!init) {
                std::cout << "Initialize" << std::endl;
                init = true;
                lat0 = p.lat;
                lon0 = p.lon;
                h0 = p.alt;
                scale = cos(lat0 * geodetic::pi / 180.0);
                // std::vector<double> t = geodetic::geodetic2enu(p.lat, p.lon, p.alt, lat0, lon0, h0);
                std::vector<double> t = geodetic::se3_translation(lat0, lon0, h0, scale);
                x0 = t[0];
                y0 = t[1];
                z0 = t[2];
                // std::cout <<  "scale: " << scale << " x0: " << x0 << " y0: " << y0 << " z0: " << z0 << std::endl;
            }

            // std::vector<double> enu = geodetic::geodetic2enu(p.lat, p.lon, p.alt, lat0, lon0, h0);
            std::vector<double> enu = geodetic::se3_translation(p.lat, p.lon, p.alt, scale);
            p.x = enu[0] - x0;
            p.y = enu[1] - y0;
            p.z = enu[2] - z0;

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
        // std::string write_name = "/home/xinyuwang/adehome/kitti_to_ros2bag/velodyne_points/pcd/" + pcd_mapping::pad_name(i) + ".pcd";
        std::string pcd_name = pcd_path + "/" + pcd_mapping::pad_name(i) + ".bin";
        Ptr cloud = pcd_mapping::read_cloud(pcd_name);
        // pcl::io::savePCDFileASCII(write_name, *cloud);
        Ptr filtered_cloud (new PointCloud);
        pcl::ApproximateVoxelGrid<PointType> filter;
        filter.setLeafSize (0.2, 0.2, 0.2);
        filter.setInputCloud (cloud);
        filter.filter (*filtered_cloud);
        pcd_mapping::Pose p = poses[i];
        // Eigen::AngleAxisf rollAngle_m(p.roll, Eigen::Vector3f::UnitX());
        // Eigen::AngleAxisf pitchAngle_m(p.pitch, Eigen::Vector3f::UnitY());
        // Eigen::AngleAxisf yawAngle_m(p.yaw, Eigen::Vector3f::UnitZ());

        // Eigen::Quaternion<float> q = yawAngle * (pitchAngle * rollAngle);

        // Eigen::Matrix3f rotation = q.matrix();

        // if(i > 0) {
        // rotation.setIdentity();
        // }

        Eigen::Matrix3f rollAngle = geodetic::rotx(p.roll);
        Eigen::Matrix3f pitchAngle = geodetic::roty(p.pitch);
        Eigen::Matrix3f yawAngle = geodetic::rotz(p.yaw);

        Eigen::Matrix3f rotation = yawAngle * (pitchAngle * rollAngle);

        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block<3,3>(0,0) = rotation;
        Eigen::Vector3f translation(p.x, p.y, p.z);

        // std::cout << translation << std::endl;

        trans.block<3,1>(0,3) = translation;
        
        // std::cout << trans << std::endl;

        // trans.setIdentity();
        // Ptr trans_cloud(new PointCloud);
        // pcl::transformPointCloud(*cloud, *trans_cloud, trans);
        // if(map->size() == 0) {
        Ptr trans_cloud(new PointCloud);
        pcl::transformPointCloud(*filtered_cloud, *trans_cloud, trans);
        *map += *trans_cloud;
        // }
        // else {
        //     pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        //     // pclomp::NormalDistributionsTransform<PointType, PointType> ndt;
        //     ndt.setTransformationEpsilon (0.05);
        //     ndt.setStepSize (0.1);
        //     ndt.setResolution (0.5);
        //     ndt.setMaximumIterations (20);
        //     ndt.setInputSource (filtered_cloud);
        //     ndt.setInputTarget (map);
        //     // ndt.setNumThreads(8);
        //     Ptr out_cloud(new PointCloud);
        //     ndt.align(*out_cloud, trans);
        //     pcl::transformPointCloud(*filtered_cloud, *out_cloud, ndt.getFinalTransformation());
        //     *map += *out_cloud;

        // }

        // std::cout << "Map size : " << map->size() << std::endl;

        // std::string map_path = "/home/xinyuwang/adehome/kitti_to_ros2bag/test/";
        // Ptr filtered_map (new PointCloud);
        // pcl::VoxelGrid<PointType> voxel_filter;
        // voxel_filter.setLeafSize (1.0, 1.0, 1.0);
        // voxel_filter.setInputCloud (map);
        // voxel_filter.filter (*filtered_map);
        // std::string write_name = map_path + std::to_string(i) + ".pcd";
        // pcl::io::savePCDFileASCII(write_name, *filtered_map);

    }

    // std::cout << "Pose size: " << poses.size() << std::endl;
    return true;
}

int main() {
    std::vector<std::string> pose_paths{
        "/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_26/2011_09_26/2011_09_26_drive_0023_sync/oxts/data"};
    std::vector<std::string> pcd_paths{
        "/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_26/2011_09_26/2011_09_26_drive_0023_sync/velodyne_points/data"};
    
    // std::string pose_path = "/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_28/2011_09_28/2011_09_28_drive_0001_sync/oxts/data";

    // std::string pcd_path = "/home/xinyuwang/adehome/kitti_bag/kitti_raw/2011_09_28/2011_09_28/2011_09_28_drive_0001_sync/velodyne_points/data";

    Ptr map(new PointCloud);

    for(int i=0; i < pose_paths.size(); i++) {
        if(!process_track(pose_paths[i], pcd_paths[i], map)) {
            std::cerr << pose_paths[i] << " Failed" << std::endl;
        }
        Ptr filtered_map (new PointCloud);
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setLeafSize (0.4, 0.4, 0.4);
        voxel_filter.setInputCloud (map);
        voxel_filter.filter (*filtered_map);
        map.swap(filtered_map);
        // pcl::io::savePCDFileASCII("/home/xinyuwang/adehome/kitti_to_ros2bag/map.pcd", *filtered_map);
        std::cout << "After map size: " << map->size() << std::endl;
    }

    pcl::io::savePCDFileASCII("/home/xinyuwang/adehome/kitti_to_ros2bag/map.pcd", *map);

    return 0;
}