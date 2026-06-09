#ifndef MAP_POINT_CLOUD_HPP
#define MAP_POINT_CLOUD_HPP

#include <fstream>
#include <string>
#include <vector>

#include "MapPoint.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

namespace MapPointCloud
{

inline std::vector<Eigen::Vector3f> CollectValidPoints(
    const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(map_points.size());
    for (auto* pMP : map_points) {
        if (!pMP || pMP->isBad()) {
            continue;
        }
        pts.push_back(pMP->GetWorldPos());
    }
    return pts;
}

inline sensor_msgs::msg::PointCloud2 ToPointCloud2(
    const std::vector<Eigen::Vector3f>& pts,
    const std_msgs::msg::Header& header)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = pts.size();
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pts.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto& p : pts) {
        *iter_x = p.x();
        *iter_y = p.y();
        *iter_z = p.z();
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    return cloud;
}

inline bool SaveToPCD(
    const std::string& filename,
    const std::vector<Eigen::Vector3f>& pts)
{
    std::ofstream f(filename);
    if (!f.is_open()) {
        return false;
    }

    f << "# .PCD v0.7 - Point Cloud Data file format\n";
    f << "VERSION 0.7\n";
    f << "FIELDS x y z\n";
    f << "SIZE 4 4 4\n";
    f << "TYPE F F F\n";
    f << "COUNT 1 1 1\n";
    f << "WIDTH " << pts.size() << "\n";
    f << "HEIGHT 1\n";
    f << "VIEWPOINT 0 0 0 1 0 0 0\n";
    f << "POINTS " << pts.size() << "\n";
    f << "DATA ascii\n";
    for (const auto& p : pts) {
        f << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    return true;
}

}  // namespace MapPointCloud

#endif
