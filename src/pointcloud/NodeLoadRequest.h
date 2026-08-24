#pragma once

#include "pointcloud/BoundingBox.h"
#include "pointcloud/OctreeNode.h"

#include <cstdint>
#include <string>

struct NodeLoadRequest {
    std::uint64_t datasetGeneration = 0;
    std::string nodeId;
    std::uint64_t requestGeneration = 0;
    std::uint32_t level = 0;
    BoundingBox bounds;
    std::uint64_t pointCount = 0;
    OctreeNodeType type = OctreeNodeType::Leaf;
    HierarchyState hierarchyState = HierarchyState::Resolved;
    std::uint64_t hierarchyByteOffset = 0;
    std::uint64_t hierarchyByteSize = 0;
    std::uint64_t pointByteOffset = 0;
    std::uint64_t pointByteSize = 0;
    double requestWeight = 0.0;
    bool hierarchyOnly = false;
};
