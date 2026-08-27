#include "pointcloud/LodSelector.h"

#include <osg/Vec4d>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace
{
osg::Vec3d boundsCenter(const BoundingBox& bounds)
{
    return (bounds.min + bounds.max) * 0.5;
}

double boundsRadius(const BoundingBox& bounds)
{
    return (bounds.max - boundsCenter(bounds)).length();
}

std::array<osg::Vec3d, 8> boundsCorners(const BoundingBox& bounds)
{
    return {
        osg::Vec3d(bounds.min.x(), bounds.min.y(), bounds.min.z()),
        osg::Vec3d(bounds.max.x(), bounds.min.y(), bounds.min.z()),
        osg::Vec3d(bounds.min.x(), bounds.max.y(), bounds.min.z()),
        osg::Vec3d(bounds.max.x(), bounds.max.y(), bounds.min.z()),
        osg::Vec3d(bounds.min.x(), bounds.min.y(), bounds.max.z()),
        osg::Vec3d(bounds.max.x(), bounds.min.y(), bounds.max.z()),
        osg::Vec3d(bounds.min.x(), bounds.max.y(), bounds.max.z()),
        osg::Vec3d(bounds.max.x(), bounds.max.y(), bounds.max.z()),
    };
}

struct QueueItem {
    const OctreeNode* node = nullptr;
    double weight = 0.0;
    std::uint64_t sequence = 0;
};

struct QueueItemLess {
    bool operator()(const QueueItem& lhs, const QueueItem& rhs) const
    {
        if (lhs.weight == rhs.weight) {
            return lhs.sequence > rhs.sequence;
        }
        return lhs.weight < rhs.weight;
    }
};
} // namespace

SelectionResult LodSelector::select(const OctreeNode& root,
                                    const CameraState& camera,
                                    const LodSelectionSettings& settings) const
{
    SelectionResult result;
    // 权重是节点在屏幕上的投影半径；priority_queue 让最重要的节点先占用预算。
    std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemLess> queue;
    std::uint64_t sequence = 0;
    queue.push({&root, std::numeric_limits<double>::infinity(), sequence++});

    while (!queue.empty()) {
        const QueueItem item = queue.top();
        queue.pop();

        const OctreeNode* node = item.node;
        if (!node) {
            continue;
        }

        // The root is the bootstrap/fallback node. It must remain selectable
        // before any resident geometry gives the camera manipulator a bound.
        if (node != &root && !intersectsFrustum(*node, camera)) {
            continue;
        }
        if (node->level > settings.maxLevel) {
            continue;
        }
        if (node != &root
            && result.selectedPointCount + node->pointCount > settings.pointBudget) {
            continue;
        }

        if (node == &root
            && result.selectedPointCount + node->pointCount > settings.pointBudget) {
            result.overBudget = true;
        }

        // selected 表示“本帧希望看到”，不要求节点此刻已经 resident/CpuReady。
        result.selectedPointCount += node->pointCount;
        result.highestSelectedLevel = std::max(result.highestSelectedLevel, node->level);
        const bool unresolvedHierarchy = node->type == OctreeNodeType::Proxy
            && node->hierarchyState != HierarchyState::Resolved;

        result.selectedNodes.push_back({
            node->id,
            item.weight,
            node->pointCount,
            node->gpuState == GpuState::Resident,
            node->pointDataState == PointDataState::CpuReady,
            unresolvedHierarchy,
        });

        if (unresolvedHierarchy) {
            // Proxy 的子结构未知：先把它作为 fallback 选中并请求 hierarchy，当前
            // 遍历不能继续进入尚不存在的 children。
            if (node->hierarchyState == HierarchyState::Proxy) {
                result.loadCandidates.push_back({node->id, item.weight, true});
            }
            continue;
        }

        if (node->pointDataState == PointDataState::Unloaded) {
            // 选择器只报告候选，不改变状态；Runtime 稍后按 outstanding 上限提交。
            result.loadCandidates.push_back({node->id, item.weight, false});
        }

        for (const std::unique_ptr<OctreeNode>& child : node->children) {
            if (!child) {
                continue;
            }
            const double childWeight = projectedPixelRadius(*child, camera);
            // 屏幕投影过小的子节点不再细分，减少远处节点遍历和加载。
            if (childWeight >= settings.minimumNodePixelSize) {
                queue.push({child.get(), childWeight, sequence++});
            }
        }
    }

    return result;
}

double LodSelector::projectedPixelRadius(const OctreeNode& node,
                                         const CameraState& camera)
{
    const osg::Vec3d center = boundsCenter(node.bounds);
    const double radius = boundsRadius(node.bounds);
    const double distance = (camera.position - center).length();
    if (distance <= radius) {
        return std::numeric_limits<double>::infinity();
    }

    const double projectionScale = camera.projectionMatrix(1, 1);
    if (projectionScale <= 0.0 || camera.viewportHeight <= 0) {
        return 0.0;
    }

    return radius * static_cast<double>(camera.viewportHeight) * projectionScale / (2.0 * distance);
}

bool LodSelector::intersectsFrustum(const OctreeNode& node,
                                    const CameraState& camera)
{
    const osg::Matrixd viewProjection = camera.viewMatrix * camera.projectionMatrix;
    const std::array<osg::Vec3d, 8> corners = boundsCorners(node.bounds);

    bool outsideLeft = true;
    bool outsideRight = true;
    bool outsideBottom = true;
    bool outsideTop = true;
    bool outsideNear = true;
    bool outsideFar = true;

    for (const osg::Vec3d& corner : corners) {
        const osg::Vec4d clip = osg::Vec4d(corner, 1.0) * viewProjection;
        outsideLeft = outsideLeft && clip.x() < -clip.w();
        outsideRight = outsideRight && clip.x() > clip.w();
        outsideBottom = outsideBottom && clip.y() < -clip.w();
        outsideTop = outsideTop && clip.y() > clip.w();
        outsideNear = outsideNear && clip.z() < -clip.w();
        outsideFar = outsideFar && clip.z() > clip.w();
    }

    return !(outsideLeft || outsideRight || outsideBottom || outsideTop || outsideNear || outsideFar);
}
