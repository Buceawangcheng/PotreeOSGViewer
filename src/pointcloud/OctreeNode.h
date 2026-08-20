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

enum class HierarchyState {
    Resolved,
    Proxy,
    Queued,
    Loading,
    Failed
};

enum class PointDataState {
    Unloaded,
    Queued,
    Loading,
    CpuReady,
    Failed
};

enum class GpuState {
    Detached,
    Resident
};

struct OctreeNode {
    std::string id;
    std::uint32_t level = 0;
    BoundingBox bounds;
    std::uint64_t pointCount = 0;
    std::uint8_t childMask = 0;
    OctreeNodeType type = OctreeNodeType::Leaf;
    HierarchyState hierarchyState = HierarchyState::Resolved;
    PointDataState pointDataState = PointDataState::Unloaded;
    GpuState gpuState = GpuState::Detached;
    std::uint64_t hierarchyByteOffset = 0;
    std::uint64_t hierarchyByteSize = 0;
    std::uint64_t pointByteOffset = 0;
    std::uint64_t pointByteSize = 0;
    std::uint64_t lastSelectedFrame = 0;
    std::uint64_t lastVisibleFrame = 0;
    std::uint64_t lastAccessFrame = 0;
    std::uint64_t requestGeneration = 0;
    double selectionWeight = 0.0;
    double requestWeight = 0.0;
    std::uint64_t lastRequestedFrame = 0;
    std::uint64_t cpuBytes = 0;
    std::uint64_t gpuBytes = 0;
    std::shared_ptr<PointCloudNodeData> data;
    std::array<std::unique_ptr<OctreeNode>, 8> children;
};
