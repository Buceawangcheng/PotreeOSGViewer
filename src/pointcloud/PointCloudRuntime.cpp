#include "pointcloud/PointCloudRuntime.h"

#include "pointcloud/Potree2Provider.h"
#include "viewer/SceneManager.h"

#include <osg/Camera>
#include <osg/Viewport>
#include <osgViewer/Viewer>

#include <QDebug>

#include <algorithm>
#include <vector>

namespace
{
OctreeNode* findNodeRecursive(OctreeNode* node, const std::string& nodeId)
{
    if (!node) {
        return nullptr;
    }
    if (node->id == nodeId) {
        return node;
    }
    for (std::unique_ptr<OctreeNode>& child : node->children) {
        if (OctreeNode* found = findNodeRecursive(child.get(), nodeId)) {
            return found;
        }
    }
    return nullptr;
}

template<typename Fn>
void visitNodes(OctreeNode* node, Fn&& fn)
{
    if (!node) {
        return;
    }
    fn(*node);
    for (std::unique_ptr<OctreeNode>& child : node->children) {
        visitNodes(child.get(), fn);
    }
}

double elapsedMs(std::chrono::steady_clock::time_point start,
                 std::chrono::steady_clock::time_point finish)
{
    return std::chrono::duration<double, std::milli>(finish - start).count();
}
} // namespace

PointCloudRuntime::PointCloudRuntime(SceneManager* sceneManager)
    : m_sceneManager(sceneManager)
{
    m_scheduler.setMaxConcurrentLoads(4);
    m_settings.pointBudget = 1000000;
    m_settings.minimumNodePixelSize = 30.0;
}

void PointCloudRuntime::setCompletionCallback(std::function<void()> callback)
{
    m_scheduler.setCompletionCallback(std::move(callback));
}

void PointCloudRuntime::openDataset(std::shared_ptr<PointCloudDataset> dataset, float pointSize)
{
    ++m_generation;
    m_dataset = std::move(dataset);
    m_frame = 0;
    m_stats = PointCloudRuntimeStats {};

    if (m_dataset) {
        m_settings.maxLevel = m_dataset->hierarchyDepth;
        m_residentPointLimit = m_settings.pointBudget * 2;
        m_scheduler.setDataset(m_dataset);
        if (m_sceneManager) {
            m_sceneManager->beginPotreeLayer(*m_dataset, pointSize);
        }
    } else {
        m_scheduler.clear();
        if (m_sceneManager) {
            m_sceneManager->clear();
        }
    }
}

void PointCloudRuntime::clear()
{
    ++m_generation;
    m_scheduler.clear();
    m_dataset.reset();
    m_frame = 0;
    m_stats = PointCloudRuntimeStats {};
}

void PointCloudRuntime::update(osgViewer::Viewer* viewer, float pointSize)
{
    if (!m_dataset || !m_dataset->root || !viewer) {
        return;
    }

    ++m_frame;
    const auto drainStart = Clock::now();
    applyCompletedResults();
    const auto drainFinish = Clock::now();

    const auto selectionStart = Clock::now();
    const SelectionResult selection = m_selector.select(*m_dataset->root,
                                                        cameraState(viewer),
                                                        m_settings);
    const auto selectionFinish = Clock::now();

    applySelection(selection);

    const auto attachStart = Clock::now();
    attachSelectedCpuReadyNodes(selection, pointSize);
    const auto attachFinish = Clock::now();

    scheduleSelectedNodes(selection);
    evictUnusedNodes();
    refreshStats(selection);

    m_stats.drainMs = elapsedMs(drainStart, drainFinish);
    m_stats.selectionMs = elapsedMs(selectionStart, selectionFinish);
    m_stats.attachMs = elapsedMs(attachStart, attachFinish);

    if ((m_frame % 60) == 0) {
        qInfo().nospace()
            << "LOD selected=" << m_stats.selectedNodeCount
            << " resident=" << m_stats.residentNodeCount
            << " queued=" << m_stats.queuedNodeCount
            << " loading=" << m_stats.loadingNodeCount
            << " cpuReady=" << m_stats.cpuReadyNodeCount
            << " selectedPoints=" << m_stats.selectedPointCount
            << " residentPoints=" << m_stats.residentPointCount
            << " level=" << m_stats.highestSelectedLevel
            << " selectMs=" << m_stats.selectionMs
            << " drainMs=" << m_stats.drainMs
            << " hierarchyMs=" << m_stats.hierarchyApplyMs
            << " attachMs=" << m_stats.attachMs;
    }
}

std::shared_ptr<PointCloudDataset> PointCloudRuntime::dataset() const
{
    return m_dataset;
}

std::uint64_t PointCloudRuntime::generation() const
{
    return m_generation;
}

const PointCloudRuntimeStats& PointCloudRuntime::stats() const
{
    return m_stats;
}

CameraState PointCloudRuntime::cameraState(osgViewer::Viewer* viewer) const
{
    CameraState state;
    osg::Camera* camera = viewer->getCamera();
    state.viewMatrix = camera->getViewMatrix();
    state.projectionMatrix = camera->getProjectionMatrix();
    state.position = osg::Vec3d(0.0, 0.0, 0.0) * osg::Matrixd::inverse(state.viewMatrix);
    if (camera->getViewport()) {
        state.viewportHeight = static_cast<int>(camera->getViewport()->height());
    }
    return state;
}

void PointCloudRuntime::applyCompletedResults()
{
    const std::vector<NodeLoadResult> completed = m_scheduler.drainCompleted();
    Potree2Provider provider;
    double hierarchyMs = 0.0;

    for (const NodeLoadResult& result : completed) {
        if (result.datasetGeneration != m_generation) {
            continue;
        }

        OctreeNode* node = findNode(result.nodeId);
        if (!node || node->requestGeneration != result.requestGeneration) {
            continue;
        }

        if (!result.error.isEmpty()) {
            if (node->hierarchyState == HierarchyState::Queued
                || node->hierarchyState == HierarchyState::Loading
                || node->hierarchyState == HierarchyState::Proxy) {
                node->hierarchyState = HierarchyState::Failed;
            }
            node->pointDataState = PointDataState::Failed;
            continue;
        }

        if (!result.hierarchyPatch.nodes.empty()) {
            const auto start = Clock::now();
            QString error;
            if (!provider.applyHierarchyPatch(m_dataset.get(), result.hierarchyPatch, &error)) {
                node->hierarchyState = HierarchyState::Failed;
                continue;
            }
            hierarchyMs += elapsedMs(start, Clock::now());
            node = findNode(result.nodeId);
            if (!node || node->requestGeneration != result.requestGeneration) {
                continue;
            }
        }

        if (!result.pointData || result.pointData->positions.size() != node->pointCount) {
            node->pointDataState = PointDataState::Failed;
            continue;
        }

        node->data = result.pointData;
        node->cpuBytes = result.pointData->cpuBytes();
        node->pointDataState = PointDataState::CpuReady;
    }

    m_stats.hierarchyApplyMs = hierarchyMs;
}

void PointCloudRuntime::applySelection(const SelectionResult& selection)
{
    for (const NodeSelection& selected : selection.selectedNodes) {
        OctreeNode* node = findNode(selected.nodeId);
        if (!node) {
            continue;
        }
        node->lastSelectedFrame = m_frame;
        node->selectionWeight = selected.weight;
    }

    visitNodes(m_dataset->root.get(), [this](OctreeNode& node) {
        if (node.gpuState != GpuState::Resident) {
            return;
        }

        const bool visible = node.lastSelectedFrame == m_frame;
        if (m_sceneManager) {
            m_sceneManager->setPotreeNodeVisible(node.id, visible);
        }
        if (visible) {
            node.lastVisibleFrame = m_frame;
            node.lastAccessFrame = m_frame;
        }
    });
}

void PointCloudRuntime::attachSelectedCpuReadyNodes(const SelectionResult& selection, float pointSize)
{
    std::size_t attachedNodes = 0;
    std::uint64_t attachedBytes = 0;

    for (const NodeSelection& selected : selection.selectedNodes) {
        if (attachedNodes >= m_maxAttachNodesPerFrame
            || attachedBytes >= m_maxAttachBytesPerFrame) {
            break;
        }

        OctreeNode* node = findNode(selected.nodeId);
        if (!node
            || node->gpuState == GpuState::Resident
            || node->pointDataState != PointDataState::CpuReady
            || !node->data) {
            continue;
        }

        const std::uint64_t bytes = node->data->cpuBytes();
        if (attachedNodes > 0 && attachedBytes + bytes > m_maxAttachBytesPerFrame) {
            break;
        }

        QString error;
        if (!m_sceneManager
            || !m_sceneManager->attachPotreeNode(
                node->id,
                node->level,
                node->bounds,
                *m_dataset,
                *node->data,
                pointSize,
                &error)) {
            node->pointDataState = PointDataState::Failed;
            continue;
        }

        node->gpuState = GpuState::Resident;
        node->gpuBytes = bytes;
        node->lastVisibleFrame = m_frame;
        node->lastAccessFrame = m_frame;
        node->data.reset();
        node->cpuBytes = 0;
        ++attachedNodes;
        attachedBytes += bytes;
    }
}

void PointCloudRuntime::scheduleSelectedNodes(const SelectionResult& selection)
{
    for (const NodeRequestCandidate& candidate : selection.loadCandidates) {
        OctreeNode* node = findNode(candidate.nodeId);
        if (!node) {
            continue;
        }

        if (candidate.hierarchyProxy) {
            if (node->hierarchyState != HierarchyState::Proxy) {
                continue;
            }
        } else if (node->pointDataState == PointDataState::Unloaded) {
            node->pointDataState = PointDataState::Queued;
        } else {
            continue;
        }

        node->requestWeight = candidate.weight;
        node->lastRequestedFrame = m_frame;
        ++node->requestGeneration;
        NodeLoadRequest request = makeRequest(*node, m_generation);
        if (candidate.hierarchyProxy) {
            node->hierarchyState = HierarchyState::Queued;
        }
        m_scheduler.schedule(std::move(request));
    }
}

void PointCloudRuntime::evictUnusedNodes()
{
    std::uint64_t residentPoints = 0;
    std::vector<OctreeNode*> residentNodes;
    visitNodes(m_dataset->root.get(), [&residentPoints, &residentNodes](OctreeNode& node) {
        if (node.gpuState == GpuState::Resident) {
            residentPoints += node.pointCount;
            residentNodes.push_back(&node);
        }
    });

    if (residentPoints <= m_residentPointLimit) {
        return;
    }

    std::sort(residentNodes.begin(), residentNodes.end(), [](const OctreeNode* lhs, const OctreeNode* rhs) {
        return lhs->lastAccessFrame < rhs->lastAccessFrame;
    });

    for (OctreeNode* node : residentNodes) {
        if (residentPoints <= m_residentPointLimit) {
            break;
        }
        if (node == m_dataset->root.get()
            || node->lastSelectedFrame == m_frame
            || node->pointDataState == PointDataState::Queued
            || node->pointDataState == PointDataState::Loading) {
            continue;
        }

        if (m_sceneManager) {
            m_sceneManager->removePotreeNode(node->id);
        }
        residentPoints -= node->pointCount;
        node->gpuState = GpuState::Detached;
        node->pointDataState = PointDataState::Unloaded;
        node->gpuBytes = 0;
        node->cpuBytes = 0;
        node->data.reset();
    }
}

void PointCloudRuntime::refreshStats(const SelectionResult& selection)
{
    m_stats.selectedPointCount = selection.selectedPointCount;
    m_stats.highestSelectedLevel = selection.highestSelectedLevel;
    m_stats.overBudget = selection.overBudget;
    m_stats.selectedNodeCount = selection.selectedNodes.size();
    m_stats.queuedNodeCount = m_scheduler.queuedCount();
    m_stats.loadingNodeCount = m_scheduler.loadingCount();
    m_stats.residentPointCount = 0;
    m_stats.residentNodeCount = 0;
    m_stats.cpuReadyNodeCount = 0;
    m_stats.cpuBytes = 0;
    m_stats.gpuBytes = 0;

    visitNodes(m_dataset->root.get(), [this](OctreeNode& node) {
        if (node.gpuState == GpuState::Resident) {
            ++m_stats.residentNodeCount;
            m_stats.residentPointCount += node.pointCount;
            m_stats.gpuBytes += node.gpuBytes;
        }
        if (node.pointDataState == PointDataState::CpuReady && node.gpuState != GpuState::Resident) {
            ++m_stats.cpuReadyNodeCount;
            m_stats.cpuBytes += node.cpuBytes;
        }
    });
}

OctreeNode* PointCloudRuntime::findNode(const std::string& nodeId) const
{
    return m_dataset ? findNodeRecursive(m_dataset->root.get(), nodeId) : nullptr;
}

NodeLoadRequest PointCloudRuntime::makeRequest(const OctreeNode& node,
                                               std::uint64_t datasetGeneration)
{
    NodeLoadRequest request;
    request.datasetGeneration = datasetGeneration;
    request.nodeId = node.id;
    request.requestGeneration = node.requestGeneration;
    request.level = node.level;
    request.bounds = node.bounds;
    request.pointCount = node.pointCount;
    request.type = node.type;
    request.hierarchyState = node.hierarchyState;
    request.hierarchyByteOffset = node.hierarchyByteOffset;
    request.hierarchyByteSize = node.hierarchyByteSize;
    request.pointByteOffset = node.pointByteOffset;
    request.pointByteSize = node.pointByteSize;
    request.requestWeight = node.requestWeight;
    return request;
}
