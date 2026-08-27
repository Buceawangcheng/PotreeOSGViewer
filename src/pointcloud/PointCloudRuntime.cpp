#include "pointcloud/PointCloudRuntime.h"

#include "pointcloud/Potree2Provider.h"
#include "viewer/SceneManager.h"

#include <osg/Camera>
#include <osg/Viewport>
#include <osgViewer/Viewer>

#include <QDebug>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
// Diagnostic switches are retained for future A/B tests and disabled by default.
constexpr bool kAttachOnlyRootForDiagnosis = false;
constexpr bool kLoadOnlyRootForDiagnosis = false;
constexpr bool kSkipHierarchyProxyLoadsForDiagnosis = false;
constexpr bool kHierarchyOnlyNonRootDiagnosis = false;
constexpr bool kPointCloudUpdateDiagnosticsEnabled = false;

OctreeNode* findNodeByPath(OctreeNode* node, const std::string& nodeId)
{
    if (!node) {
        return nullptr;
    }
    if (node->id == nodeId) {
        return node;
    }

    const std::size_t rootIdLength = node->id.size();
    if (nodeId.size() <= rootIdLength
        || nodeId.compare(0, rootIdLength, node->id) != 0) {
        return nullptr;
    }

    for (std::size_t index = rootIdLength; index < nodeId.size(); ++index) {
        const char childCharacter = nodeId[index];
        if (childCharacter < '0' || childCharacter > '7') {
            return nullptr;
        }
        node = node->children[static_cast<std::size_t>(childCharacter - '0')].get();
        if (!node) {
            return nullptr;
        }
    }

    return node->id == nodeId ? node : nullptr;
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

std::uint64_t scaledPointLimit(std::uint64_t pointBudget, std::uint64_t scale)
{
    if (scale == 0) {
        return 0;
    }
    if (pointBudget > std::numeric_limits<std::uint64_t>::max() / scale) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return pointBudget * scale;
}

bool matrixNearlyEqual(const osg::Matrixd& lhs, const osg::Matrixd& rhs)
{
    constexpr double epsilon = 1e-9;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            if (std::abs(lhs(row, column) - rhs(row, column)) > epsilon) {
                return false;
            }
        }
    }
    return true;
}

bool cameraStateNearlyEqual(const CameraState& lhs, const CameraState& rhs)
{
    constexpr double epsilon = 1e-9;
    return lhs.viewportHeight == rhs.viewportHeight
        && (lhs.position - rhs.position).length2() <= epsilon * epsilon
        && matrixNearlyEqual(lhs.viewMatrix, rhs.viewMatrix)
        && matrixNearlyEqual(lhs.projectionMatrix, rhs.projectionMatrix);
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
    rebuildNodeIndex();
    m_hasLastCameraState = false;
    m_lastSelection = SelectionResult {};
    m_lastSlowUpdateLog = Clock::time_point {};

    if (m_dataset) {
        if constexpr (kAttachOnlyRootForDiagnosis) {
            qWarning().noquote()
                << "Potree A/B diagnostic enabled: only the root node will be attached to OSG.";
        }
        if constexpr (kLoadOnlyRootForDiagnosis) {
            qWarning().noquote()
                << "Potree A/B diagnostic stage 2 enabled: non-root loading, CPU decoding "
                   "and hierarchy patching are disabled.";
        }
        if constexpr (kSkipHierarchyProxyLoadsForDiagnosis) {
            qWarning().noquote()
                << "Potree A/B diagnostic stage 4 enabled: hierarchy proxy requests are disabled; "
                   "ordinary point IO and CPU decoding remain enabled.";
        }
        if constexpr (kHierarchyOnlyNonRootDiagnosis) {
            qWarning().noquote()
                << "Potree A/B diagnostic stage 5 enabled: non-root hierarchy is resolved, "
                   "but non-root point IO and CPU decoding are disabled.";
        }
        m_settings.maxLevel = m_dataset->hierarchyDepth;
        m_residentPointTargetLimit = scaledPointLimit(m_settings.pointBudget, 6);
        m_residentPointLimit = scaledPointLimit(m_settings.pointBudget, 8);
        m_maxEvictPointsPerFrame = std::max<std::uint64_t>(
            m_settings.pointBudget / 2, 500000ull);
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
    m_nodeIndex.clear();
    m_hasLastCameraState = false;
    m_lastSelection = SelectionResult {};
    m_lastSlowUpdateLog = Clock::time_point {};
}

void PointCloudRuntime::update(osgViewer::Viewer* viewer, float pointSize)
{
    if (!m_dataset || !m_dataset->root || !viewer) {
        return;
    }

    // 本函数运行在 OSG update traversal。完整流水线是：
    // 1. drain 工作线程结果并在主线程落地；
    // 2. 相机/hierarchy 未变化时复用上次 LOD，只继续 attach 和补请求；
    // 3. 否则重新遍历 Octree 选择 LOD并更新 resident 节点可见性；
    // 4. 按预算把 CpuReady 节点挂到 OSG，随后补充异步请求；
    // 5. resident cache 超预算时分帧淘汰，之后由 OSG 继续 cull/draw traversal。
    ++m_frame;
    const auto updateStart = Clock::now();
    const auto drainStart = Clock::now();
    //接收后台线程已经完成的结果，把节点更新为 CpuReady
    const bool hierarchyChanged = applyCompletedResults();
    const auto drainFinish = Clock::now();

    const CameraState currentCamera = cameraState(viewer);
    // 异步结果会改变 CpuReady/Queued 状态，但只要没有新增 hierarchy，节点集合和
    // 相机都不变，就无需再次执行 LOD/全树可见性/淘汰遍历。
    if (!hierarchyChanged
        && m_hasLastCameraState
        && cameraStateNearlyEqual(currentCamera, m_lastCameraState)
        && !m_lastSelection.selectedNodes.empty()) {
        const auto attachStart = Clock::now();
        //将 CPU-ready 数据挂到 OSG/GPU，每帧限制 2 个节点或 32 MB
        attachSelectedCpuReadyNodes(m_lastSelection, pointSize, false);
        const auto attachFinish = Clock::now();
        //维持最多 4 个后台加载任务，不断为完成后的空位补充新任务
        scheduleSelectedNodes(m_lastSelection);

        m_stats.drainMs = elapsedMs(drainStart, drainFinish);
        m_stats.selectionMs = 0.0;
        m_stats.attachMs = elapsedMs(attachStart, attachFinish);
        m_stats.evictMs = 0.0;
        m_stats.queuedNodeCount = m_scheduler.queuedCount();
        m_stats.loadingNodeCount = m_scheduler.loadingCount();
        return;
    }

    // 阶段 2：纯 CPU best-first 遍历，生成本帧目标节点和加载候选；不做 IO。
    const auto selectionStart = Clock::now();
    const SelectionResult selection = m_selector.select(*m_dataset->root,
                                                        currentCamera,
                                                        m_settings);
    const auto selectionFinish = Clock::now();
    m_lastCameraState = currentCamera;
    m_hasLastCameraState = true;
    m_lastSelection = selection;

    // 阶段 3：把 selected 映射为已有 resident OSG 节点的显示/隐藏状态。
    const auto applySelectionStart = Clock::now();
    applySelection(selection);
    const auto applySelectionFinish = Clock::now();

    // 阶段 4：只挂载已经 CpuReady 的 selected 节点，并受单帧预算限制。
    const auto attachStart = Clock::now();
    attachSelectedCpuReadyNodes(selection, pointSize, true);
    const auto attachFinish = Clock::now();

    // 阶段 5：对 selected 中仍 Unloaded/Proxy 的节点制作值快照并提交后台。
    const auto scheduleStart = Clock::now();
    scheduleSelectedNodes(selection);
    const auto scheduleFinish = Clock::now();
    // 阶段 6：渲染预算与 resident cache 分离；仅在缓存超过上限时回收旧节点。
    const auto evictStart = Clock::now();
    evictUnusedNodes();
    const auto evictFinish = Clock::now();
    //refreshStats(selection);

    /*m_stats.drainMs = elapsedMs(drainStart, drainFinish);
    m_stats.selectionMs = elapsedMs(selectionStart, selectionFinish);
    m_stats.attachMs = elapsedMs(attachStart, attachFinish);
    m_stats.evictMs = elapsedMs(evictStart, evictFinish);*/

    if constexpr (kPointCloudUpdateDiagnosticsEnabled) {
        const auto updateFinish = Clock::now();
        const double updateMs = elapsedMs(updateStart, updateFinish);
        const double drainMs = elapsedMs(drainStart, drainFinish);
        const double selectionMs = elapsedMs(selectionStart, selectionFinish);
        const double applySelectionMs = elapsedMs(applySelectionStart, applySelectionFinish);
        const double scheduleMs = elapsedMs(scheduleStart, scheduleFinish);
        const double attachMs = elapsedMs(attachStart, attachFinish);
        const double evictMs = elapsedMs(evictStart, evictFinish);
        const double sinceLastLogMs = m_lastSlowUpdateLog == Clock::time_point {}
            ? std::numeric_limits<double>::infinity()
            : elapsedMs(m_lastSlowUpdateLog, updateFinish);
        const bool hierarchyApplied = m_stats.hierarchyNodeCount > 0;
        if ((hierarchyApplied || updateMs >= 2.0) && sinceLastLogMs >= 500.0) {
            m_lastSlowUpdateLog = updateFinish;
            qWarning().nospace()
                << "PointCloud Update diagnostic frame=" << m_frame
                << " totalMs=" << updateMs
                << " drainMs=" << drainMs
                << " hierarchyApplyMs=" << m_stats.hierarchyApplyMs
                << " hierarchyIndexMs=" << m_stats.hierarchyIndexMs
                << " hierarchyNodes=" << m_stats.hierarchyNodeCount
                << " selectMs=" << selectionMs
                << " applySelectionMs=" << applySelectionMs
                << " scheduleMs=" << scheduleMs
                << " attachMs=" << attachMs
                << " evictMs=" << evictMs
                << " selectedNodes=" << selection.selectedNodes.size()
                << " loadCandidates=" << selection.loadCandidates.size()
                << " outstanding=" << m_scheduler.outstandingCount();
        }
    }

    /*if ((m_frame % 60) == 0){
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
            << " hierarchyNodes=" << m_stats.hierarchyNodeCount
            << " hierarchyIndexMs=" << m_stats.hierarchyIndexMs
            << " attachMs=" << m_stats.attachMs
            << " attached=" << m_stats.attachedNodeCount
            << "/" << m_stats.attachedPointCount
            << " evictMs=" << m_stats.evictMs
            << " evicted=" << m_stats.evictedNodeCount
            << "/" << m_stats.evictedPointCount;
    }*/
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

bool PointCloudRuntime::applyCompletedResults()
{
    // drainCompleted() 用 swap 一次取走当前所有结果。下面所有 live Octree 修改均在
    // 主/update线程执行，工作线程从不直接写 node。
    const std::vector<NodeLoadResult> completed = m_scheduler.drainCompleted();
    Potree2Provider provider;
    double hierarchyMs = 0.0;
    double hierarchyIndexMs = 0.0;
    std::size_t hierarchyNodeCount = 0;
    bool hierarchyChanged = false;

    for (const NodeLoadResult& result : completed) {
        // 数据集已切换/clear：丢弃旧 dataset 的晚到结果。
        if (result.datasetGeneration != m_generation) {
            continue;
        }

        OctreeNode* node = findNode(result.nodeId);
        // 同一节点已重新请求：丢弃更早 requestGeneration 的结果。
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
            // Proxy 的 hierarchy patch 先合并到 live Octree。新增节点改变了下一次
            // LOD 可遍历的集合，因此 hierarchyChanged 会强制本帧重新选择。
            hierarchyNodeCount += result.hierarchyPatch.nodes.size();
            const auto start = Clock::now();
            QString error;
            if (!provider.applyHierarchyPatch(m_dataset.get(), result.hierarchyPatch, &error)) {
                node->hierarchyState = HierarchyState::Failed;
                continue;
            }
            hierarchyMs += elapsedMs(start, Clock::now());

            // 只为 patch 涉及的节点增量维护 id 索引，避免每个结果重扫整棵树。
            const auto indexStart = Clock::now();
            for (const HierarchyNodePatch& nodePatch : result.hierarchyPatch.nodes) {
                if (OctreeNode* patchedNode = findNodeByPath(
                        m_dataset->root.get(), nodePatch.id)) {
                    m_nodeIndex[nodePatch.id] = patchedNode;
                }
            }
            hierarchyIndexMs += elapsedMs(indexStart, Clock::now());
            hierarchyChanged = true;
            node = findNode(result.nodeId);
            if (!node || node->requestGeneration != result.requestGeneration) {
                continue;
            }
        }

        if (result.hierarchyOnly) {
            // 该诊断请求只展开结构，不包含 pointData。
            continue;
        }

        if (!result.pointData || result.pointData->positions.size() != node->pointCount) {
            node->pointDataState = PointDataState::Failed;
            continue;
        }

        // 后台值结果至此才正式进入 live node：Queued -> CpuReady。真正的 OSG
        // Geometry 创建留给 attachSelectedCpuReadyNodes() 按帧预算执行。
        node->data = result.pointData;
        node->cpuBytes = result.pointData->cpuBytes();
        node->pointDataState = PointDataState::CpuReady;
    }

    m_stats.hierarchyApplyMs = hierarchyMs;
    m_stats.hierarchyIndexMs = hierarchyIndexMs;
    m_stats.hierarchyNodeCount = hierarchyNodeCount;
    return hierarchyChanged;
}

void PointCloudRuntime::applySelection(const SelectionResult& selection)
{
    // 先用当前帧号标记本次 selected 集合。
    for (const NodeSelection& selected : selection.selectedNodes) {
        OctreeNode* node = findNode(selected.nodeId);
        if (!node) {
            continue;
        }
        node->lastSelectedFrame = m_frame;
        node->selectionWeight = selected.weight;
    }

    // 再遍历所有 resident 节点：本帧选中则显示，否则隐藏。未 resident 节点即使
    // 被 selected，也要等 CpuReady -> attach 后才能参与 cull/draw。
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

void PointCloudRuntime::attachSelectedCpuReadyNodes(const SelectionResult& selection,
                                                   float pointSize,
                                                   bool trustSelectionState)
{
    std::size_t attachedNodes = 0;
    std::uint64_t attachedBytes = 0;
    m_stats.attachedNodeCount = 0;
    m_stats.attachedPointCount = 0;

    for (const NodeSelection& selected : selection.selectedNodes) {
        // OSG Geometry/VBO 创建可能阻塞 update traversal，因此同时限制节点数和
        // CPU 数组字节数，把大量完成结果分摊到后续帧。
        if (attachedNodes >= m_maxAttachNodesPerFrame
            || attachedBytes >= m_maxAttachBytesPerFrame) {
            break;
        }
        // 新 selection 可相信快照以快速跳过；静止相机复用旧 selection 时传 false，
        // 因为后台任务可能已把当时的非 CpuReady 节点变为 CpuReady。
        if (selected.resident || (trustSelectionState && !selected.cpuReady)) {
            continue;
        }

        OctreeNode* node = findNode(selected.nodeId);
        if (!node
            || node->gpuState == GpuState::Resident
            || node->pointDataState != PointDataState::CpuReady
            || !node->data) {
            continue;
        }
        if constexpr (kAttachOnlyRootForDiagnosis) {
            if (node->id != "r") {
                continue;
            }
        }
        const std::uint64_t bytes = node->data->cpuBytes();
        if (attachedNodes > 0 && attachedBytes + bytes > m_maxAttachBytesPerFrame) {
            break;
        }

        QString error;
        // 从这里开始进入 OSG 后端；只能在主/update线程创建/挂接场景对象。
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
        // OSG arrays 已经持有自己的数据，释放临时 CPU 解码结果，节点转为 Resident。
        node->data.reset();
        node->cpuBytes = 0;
        ++attachedNodes;
        attachedBytes += bytes;
        ++m_stats.attachedNodeCount;
        m_stats.attachedPointCount += node->pointCount;
    }
}

void PointCloudRuntime::scheduleSelectedNodes(const SelectionResult& selection)
{
    // outstanding = 调度器 pending + 正在执行，限制请求积压和瞬时内存占用。
    std::size_t outstandingLoads = m_scheduler.outstandingCount();
    if (outstandingLoads >= m_maxOutstandingLoads) {
        return;
    }

    for (const NodeRequestCandidate& candidate : selection.loadCandidates) {
        if constexpr (kSkipHierarchyProxyLoadsForDiagnosis) {
            if (candidate.hierarchyProxy) {
                continue;
            }
        }

        OctreeNode* node = findNode(candidate.nodeId);
        if (!node) {
            continue;
        }
        if constexpr (kLoadOnlyRootForDiagnosis) {
            if (node->id != "r") {
                continue;
            }
        }
        if constexpr (kHierarchyOnlyNonRootDiagnosis) {
            if (!candidate.hierarchyProxy && node->id != "r") {
                continue;
            }
        }

        // 必须检查 live 状态：SelectionResult 是快照，静止相机时会跨帧复用。
        // 状态门禁同时避免同一候选被每帧重复入队。
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
        // makeRequest() 复制全部加载字段；工作线程不会持有 node 指针。
        NodeLoadRequest request = makeRequest(*node, m_generation);
        if constexpr (kHierarchyOnlyNonRootDiagnosis) {
            request.hierarchyOnly = candidate.hierarchyProxy && node->id != "r";
        }
        if (candidate.hierarchyProxy) {
            node->hierarchyState = HierarchyState::Queued;
        }
        m_scheduler.schedule(std::move(request));
        ++outstandingLoads;
        if (outstandingLoads >= m_maxOutstandingLoads) {
            break;
        }
    }
}

void PointCloudRuntime::evictUnusedNodes()
{
    m_stats.evictedNodeCount = 0;
    m_stats.evictedPointCount = 0;

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

    const std::uint64_t hardResidentPointLimit = scaledPointLimit(m_settings.pointBudget, 12);
    std::size_t evictedNodesThisFrame = 0;
    std::uint64_t evictedPointsThisFrame = 0;

    // 最久未访问优先；当前 selected、仍有请求在途或近期可见的节点受到保护。
    std::sort(residentNodes.begin(), residentNodes.end(), [](const OctreeNode* lhs, const OctreeNode* rhs) {
        return lhs->lastAccessFrame < rhs->lastAccessFrame;
    });

    for (OctreeNode* node : residentNodes) {
        if (residentPoints <= m_residentPointTargetLimit
            || evictedNodesThisFrame >= m_maxEvictNodesPerFrame
            || (evictedNodesThisFrame > 0
                && evictedPointsThisFrame >= m_maxEvictPointsPerFrame)) {
            break;
        }
        if (node == m_dataset->root.get()
            || node->lastSelectedFrame == m_frame
            || node->pointDataState == PointDataState::Queued
            || node->pointDataState == PointDataState::Loading) {
            continue;
        }

        const bool recentlyVisible = node->lastVisibleFrame > 0
            && m_frame - node->lastVisibleFrame < m_minResidentFramesBeforeEvict;
        if (recentlyVisible && residentPoints <= hardResidentPointLimit) {
            continue;
        }

        if (m_sceneManager) {
            m_sceneManager->removePotreeNode(node->id);
        }
        residentPoints -= node->pointCount;
        ++evictedNodesThisFrame;
        evictedPointsThisFrame += node->pointCount;
        ++m_stats.evictedNodeCount;
        m_stats.evictedPointCount += node->pointCount;
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

void PointCloudRuntime::rebuildNodeIndex()
{
    m_nodeIndex.clear();
    if (!m_dataset || !m_dataset->root) {
        return;
    }

    visitNodes(m_dataset->root.get(), [this](OctreeNode& node) {
        m_nodeIndex[node.id] = &node;
    });
}

OctreeNode* PointCloudRuntime::findNode(const std::string& nodeId) const
{
    const auto indexed = m_nodeIndex.find(nodeId);
    if (indexed != m_nodeIndex.end()) {
        return indexed->second;
    }
    return m_dataset ? findNodeByPath(m_dataset->root.get(), nodeId) : nullptr;
}

NodeLoadRequest PointCloudRuntime::makeRequest(const OctreeNode& node,
                                               std::uint64_t datasetGeneration)
{
    // 跨线程只传值快照。即使 hierarchy patch 后 Octree 容器发生变化，工作线程也
    // 不会解引用失效指针；结果回来时再通过 id + generation 查找和验证 live node。
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
