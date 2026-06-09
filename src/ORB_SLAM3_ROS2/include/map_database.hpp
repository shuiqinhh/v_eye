#ifndef MAP_DATABASE_HPP
#define MAP_DATABASE_HPP

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "MapPoint.h"

namespace MapDatabase
{

constexpr char kMagic[] = "VEYEMAP1";

struct MapPointEntry
{
    float x;
    float y;
    float z;
    uint8_t descriptor[32];
};

inline std::vector<MapPointEntry> CollectEntries(
    const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    std::vector<MapPointEntry> entries;
    entries.reserve(map_points.size());
    for (auto* pMP : map_points) {
        if (!pMP || pMP->isBad()) {
            continue;
        }
        cv::Mat desc = pMP->GetDescriptor();
        if (desc.empty() || desc.cols != 32) {
            continue;
        }
        MapPointEntry entry{};
        const Eigen::Vector3f pos = pMP->GetWorldPos();
        entry.x = pos.x();
        entry.y = pos.y();
        entry.z = pos.z();
        std::memcpy(entry.descriptor, desc.ptr<uint8_t>(), 32);
        entries.push_back(entry);
    }
    return entries;
}

inline bool SaveToBinary(
    const std::string& filename,
    const std::vector<MapPointEntry>& entries)
{
    std::ofstream f(filename, std::ios::binary);
    if (!f.is_open()) {
        return false;
    }

    f.write(kMagic, 8);
    const uint32_t count = static_cast<uint32_t>(entries.size());
    f.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (const auto& entry : entries) {
        f.write(reinterpret_cast<const char*>(&entry.x), sizeof(entry.x));
        f.write(reinterpret_cast<const char*>(&entry.y), sizeof(entry.y));
        f.write(reinterpret_cast<const char*>(&entry.z), sizeof(entry.z));
        f.write(reinterpret_cast<const char*>(entry.descriptor), 32);
    }
    return f.good();
}

}  // namespace MapDatabase

#endif
