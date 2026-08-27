#pragma once

#include "pointcloud/CameraState.h"
#include "pointcloud/OctreeNode.h"

#include <cstdint>
#include <string>
#include <vector>

// 每帧 LOD 遍历的纯 CPU 输入参数。
struct LodSelectionSettings {
    std::uint64_t pointBudget = 1000000;
    double minimumNodePixelSize = 30.0;
    std::uint32_t maxLevel = 0;
};

// 一次 LOD 选择时记录的节点快照。
// resident/cpuReady 是 select() 执行瞬间的状态，不会随后台任务完成自动变化；
// 静止相机复用 SelectionResult 时，运行时会按需要重新检查 live Octree 状态。
struct NodeSelection {
    std::string nodeId;
    double weight = 0.0;
    std::uint64_t pointCount = 0;
    bool resident = false;
    bool cpuReady = false;
    bool hierarchyProxy = false;
};

// selectedNodes 中尚需异步工作的节点。hierarchyProxy=true 表示先读取
// hierarchy.bin 分块，否则直接读取/解码 octree.bin 点范围。
struct NodeRequestCandidate {
    std::string nodeId;
    double weight = 0.0;
    bool hierarchyProxy = false;
};

// LOD 遍历输出。
// selected、resident、visible 是不同概念：selected 是本帧目标；resident 表示
// 已有 OSG 节点；只有 applySelection() 才把 selected resident 节点设为可见。
struct SelectionResult {
    std::vector<NodeSelection> selectedNodes;
    std::vector<NodeRequestCandidate> loadCandidates;
    std::uint64_t selectedPointCount = 0;
    std::uint32_t highestSelectedLevel = 0;
    bool overBudget = false;
};

// 只读遍历 Octree 的 best-first LOD 选择器。
// 它不做 IO、不修改节点、不创建 OSG 对象，只根据相机、视锥、投影尺寸和点预算
// 生成 SelectionResult，异步调度由 PointCloudRuntime 负责。
class LodSelector {
public:
    SelectionResult select(const OctreeNode& root,
                           const CameraState& camera,
                           const LodSelectionSettings& settings) const;

    static double projectedPixelRadius(const OctreeNode& node,
                                       const CameraState& camera);

private:
    static bool intersectsFrustum(const OctreeNode& node,
                                  const CameraState& camera);
};
