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
    if (m_frameCallback) {
        m_frameCallback();
    }

    if (m_runtime && m_viewer && m_pointSize) {
        m_runtime->update(m_viewer, *m_pointSize);
    }

    traverse(node, visitor);
}
