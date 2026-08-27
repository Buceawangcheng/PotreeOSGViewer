#pragma once

#include "pointcloud/BoundingBox.h"
#include "pointcloud/OctreeNode.h"

#include <cstdint>
#include <string>

// 从主线程传给工作线程的不可变节点快照。
//
// 这里故意复制加载所需字段而不传 OctreeNode*：工作线程只读取文件、解析
// hierarchy 和解码点数据，不接触可能在主线程中扩展或销毁的 live Octree。
struct NodeLoadRequest {
    // 两级 generation 用来拒绝“旧数据集”或“同一节点旧请求”的晚到结果。
    std::uint64_t datasetGeneration = 0;
    std::string nodeId;
    std::uint64_t requestGeneration = 0;

    // 节点空间信息和本次请求所需的 hierarchy/point 字节范围快照。
    std::uint32_t level = 0;
    BoundingBox bounds;
    std::uint64_t pointCount = 0;
    OctreeNodeType type = OctreeNodeType::Leaf;
    HierarchyState hierarchyState = HierarchyState::Resolved;
    std::uint64_t hierarchyByteOffset = 0;
    std::uint64_t hierarchyByteSize = 0;
    std::uint64_t pointByteOffset = 0;
    std::uint64_t pointByteSize = 0;

    // projected pixel radius 等选择权重；调度器优先处理权重较高的请求。
    double requestWeight = 0.0;

    // 诊断/分阶段路径可只展开 hierarchy，不继续读取点数据。
    bool hierarchyOnly = false;
};
