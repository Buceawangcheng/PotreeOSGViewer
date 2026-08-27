#include "pointcloud/PointCloudUpdateCallback.h"

#include "pointcloud/PointCloudRuntime.h"

#include <osg/Node>
#include <osg/NodeVisitor>

#include <utility>

PointCloudUpdateCallback::PointCloudUpdateCallback(PointCloudRuntime* runtime,
                                                   osgViewer::Viewer* viewer,
                                                   float* pointSize,
                                                   std::function<void()> frameCallback)
    : m_runtime(runtime)
    , m_viewer(viewer)
    , m_pointSize(pointSize)
    , m_frameCallback(std::move(frameCallback))
{
}

void PointCloudUpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* visitor)
{
    // 先记录即将渲染的一帧，再在 update traversal 中推进流式状态机。
    if (m_frameCallback) {
        m_frameCallback();
    }

    if (m_runtime && m_viewer && m_pointSize) {
        m_runtime->update(m_viewer, *m_pointSize);
    }

    // Runtime 新挂载/隐藏/移除的 OSG 节点随后立即参与本帧剩余 traversal，最终由
    // OSG 的 cull/draw 阶段提交 OpenGL 绘制。
    traverse(node, visitor);
}
