#pragma once

#include "pointcloud/BoundingBox.h"
#include "pointcloud/OctreeNode.h"

#include <cstdint>
#include <string>
#include <vector>

// 工作线程从一个 hierarchy.bin 分块解析出的单节点描述。
// 它是可跨线程传递的值，不包含 live OctreeNode 指针。
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

// 一个 Proxy 节点对应的 hierarchy 分块解析结果。
// PointCloudRuntime 在主线程调用 applyHierarchyPatch() 创建/更新真实 OctreeNode，
// 然后增量更新 nodeId -> OctreeNode* 索引并重新执行一次 LOD 选择。
struct HierarchyPatch {
    std::string rootNodeId;
    std::uint64_t chunkByteOffset = 0;
    std::uint64_t chunkByteSize = 0;
    std::vector<HierarchyNodePatch> nodes;
};
