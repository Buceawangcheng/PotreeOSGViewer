#pragma once

#include "pointcloud/LodSelector.h"
#include "pointcloud/NodeLoadScheduler.h"
#include "pointcloud/PointCloudDataset.h"

#include <QString>

#include <chrono>
#include <functional>
#include <memory>
#include <unordered_map>

namespace osgViewer {
class Viewer;
}

class SceneManager;

struct PointCloudRuntimeStats {
    double selectionMs = 0.0;
    double drainMs = 0.0;
    double hierarchyApplyMs = 0.0;
    double hierarchyIndexMs = 0.0;
    double attachMs = 0.0;
    double evictMs = 0.0;
    std::uint64_t selectedPointCount = 0;
    std::uint64_t residentPointCount = 0;
    std::uint64_t attachedPointCount = 0;
    std::uint64_t evictedPointCount = 0;
    std::uint64_t cpuBytes = 0;
    std::uint64_t gpuBytes = 0;
    std::size_t selectedNodeCount = 0;
    std::size_t residentNodeCount = 0;
    std::size_t hierarchyNodeCount = 0;
    std::size_t attachedNodeCount = 0;
    std::size_t evictedNodeCount = 0;
    std::size_t queuedNodeCount = 0;
    std::size_t loadingNodeCount = 0;
    std::size_t cpuReadyNodeCount = 0;
    std::uint32_t highestSelectedLevel = 0;
    bool overBudget = false;
};

// 点云流式运行时的主线程编排器。
//
// 每次 OSG update traversal 中，它依次接收后台结果、选择 LOD、更新可见性、
// 分帧挂载 CPU-ready 节点、补充异步请求并按缓存预算淘汰节点。它是 live Octree
// 状态和 OSG 场景修改的唯一入口；NodeLoadScheduler 的工作线程只返回值对象。
class PointCloudRuntime {
public:
    explicit PointCloudRuntime(SceneManager* sceneManager);

    void setCompletionCallback(std::function<void()> callback);
    void openDataset(std::shared_ptr<PointCloudDataset> dataset, float pointSize);
    void clear();
    void update(osgViewer::Viewer* viewer, float pointSize);

    std::shared_ptr<PointCloudDataset> dataset() const;
    std::uint64_t generation() const;
    const PointCloudRuntimeStats& stats() const;

private:
    using Clock = std::chrono::steady_clock;

    // update() 主流程拆分出的各阶段，调用顺序见 PointCloudRuntime.cpp。
    CameraState cameraState(osgViewer::Viewer* viewer) const;
    // 过滤晚到结果，在主线程应用 hierarchy patch，并把点数组标记为 CpuReady。
    bool applyCompletedResults();
    // 把 LOD 选择映射为 resident OSG 节点的可见/隐藏状态。
    void applySelection(const SelectionResult& selection);
    // 按单帧节点数/字节预算，将 CpuReady 数据转换为 OSG Geometry。
    void attachSelectedCpuReadyNodes(const SelectionResult& selection,
                                     float pointSize,
                                     bool trustSelectionState);
    // 将选中但未加载的节点快照提交到有界后台调度器。
    void scheduleSelectedNodes(const SelectionResult& selection);
    // resident cache 超预算时，按最近访问时间分帧移除 OSG 节点。
    void evictUnusedNodes();
    void refreshStats(const SelectionResult& selection);
    void rebuildNodeIndex();

    OctreeNode* findNode(const std::string& nodeId) const;
    static NodeLoadRequest makeRequest(const OctreeNode& node,
                                       std::uint64_t datasetGeneration);

    SceneManager* m_sceneManager = nullptr; // 非拥有；仅在主/update线程调用。
    std::shared_ptr<PointCloudDataset> m_dataset;
    NodeLoadScheduler m_scheduler;
    LodSelector m_selector;
    LodSelectionSettings m_settings;
    PointCloudRuntimeStats m_stats;
    // 主线程索引；hierarchy patch 创建节点后增量更新，避免反复递归查找整棵树。
    std::unordered_map<std::string, OctreeNode*> m_nodeIndex;
    CameraState m_lastCameraState;
    SelectionResult m_lastSelection;
    Clock::time_point m_lastSlowUpdateLog;
    // 数据集切换/clear 时递增，使旧工作线程结果失效。
    std::uint64_t m_generation = 0;
    std::uint64_t m_frame = 0;
    bool m_hasLastCameraState = false;
    std::uint64_t m_residentPointLimit = 8000000;
    std::uint64_t m_residentPointTargetLimit = 6000000;
    std::size_t m_maxAttachNodesPerFrame = 2;
    std::uint64_t m_maxAttachBytesPerFrame = 32ull * 1024ull * 1024ull;
    std::size_t m_maxOutstandingLoads = 4;
    std::size_t m_maxEvictNodesPerFrame = 4;
    std::uint64_t m_maxEvictPointsPerFrame = 500000;
    std::uint64_t m_minResidentFramesBeforeEvict = 180;
};
