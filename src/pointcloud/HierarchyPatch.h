#pragma once

#include "pointcloud/BoundingBox.h"
#include "pointcloud/OctreeNode.h"

#include <cstdint>
#include <string>
#include <vector>

struct HierarchyNodePatch {
    std::string id;
    std::uint32_t level = 0;
    BoundingBox bounds;
    std::uint64_t pointCount = 0;
    std::uint8_t childMask = 0;
    OctreeNodeType type = OctreeNodeType::Leaf;
    std::uint64_t hierarchyByteOffset = 0;
    std::uint64_t hierarchyByteSize = 0;
    std::uint64_t pointByteOffset = 0;
    std::uint64_t pointByteSize = 0;
};

struct HierarchyPatch {
    std::string rootNodeId;
    std::uint64_t chunkByteOffset = 0;
    std::uint64_t chunkByteSize = 0;
    std::vector<HierarchyNodePatch> nodes;
};
