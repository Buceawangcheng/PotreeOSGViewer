#pragma once

#include "pointcloud/BoundingBox.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

struct PointCloudNodeData;

enum class OctreeNodeType {
    Normal,
    Leaf,
    Proxy
};

enum class OctreeNodeLoadState {
    Unloaded,
    Queued,
    Loading,
    CpuReady,
    Attached,
    Failed,
    Evicting
};

struct OctreeNode {
    std::string id;
    std::uint32_t level = 0;
    BoundingBox bounds;
    std::uint64_t pointCount = 0;
    std::uint8_t childMask = 0;
    OctreeNodeType type = OctreeNodeType::Leaf;
    OctreeNodeLoadState loadState = OctreeNodeLoadState::Unloaded;
    std::uint64_t byteOffset = 0;
    std::uint64_t byteSize = 0;
    std::uint64_t lastAccessFrame = 0;
    std::uint64_t cpuBytes = 0;
    std::uint64_t gpuBytes = 0;
    std::shared_ptr<PointCloudNodeData> data;
    std::array<std::unique_ptr<OctreeNode>, 8> children;
};
