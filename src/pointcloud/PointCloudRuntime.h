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

    CameraState cameraState(osgViewer::Viewer* viewer) const;
    bool applyCompletedResults();
    void applySelection(const SelectionResult& selection);
    void attachSelectedCpuReadyNodes(const SelectionResult& selection,
                                     float pointSize,
                                     bool trustSelectionState);
    void scheduleSelectedNodes(const SelectionResult& selection);
    void evictUnusedNodes();
    void refreshStats(const SelectionResult& selection);
    void rebuildNodeIndex();

    OctreeNode* findNode(const std::string& nodeId) const;
    static NodeLoadRequest makeRequest(const OctreeNode& node,
                                       std::uint64_t datasetGeneration);

    SceneManager* m_sceneManager = nullptr;
    std::shared_ptr<PointCloudDataset> m_dataset;
    NodeLoadScheduler m_scheduler;
    LodSelector m_selector;
    LodSelectionSettings m_settings;
    PointCloudRuntimeStats m_stats;
    std::unordered_map<std::string, OctreeNode*> m_nodeIndex;
    CameraState m_lastCameraState;
    SelectionResult m_lastSelection;
    std::uint64_t m_generation = 0;
    std::uint64_t m_frame = 0;
    bool m_hasLastCameraState = false;
    std::uint64_t m_residentPointLimit = 8000000;
    std::uint64_t m_residentPointTargetLimit = 6000000;
    std::size_t m_maxAttachNodesPerFrame = 2;
    std::uint64_t m_maxAttachBytesPerFrame = 32ull * 1024ull * 1024ull;
    std::size_t m_maxEvictNodesPerFrame = 4;
    std::uint64_t m_maxEvictPointsPerFrame = 500000;
    std::uint64_t m_minResidentFramesBeforeEvict = 180;
};
