#include "pointcloud/PointCloudUpdateCallback.h"

#include "pointcloud/PointCloudRuntime.h"

#include <osg/Node>
#include <osg/NodeVisitor>

PointCloudUpdateCallback::PointCloudUpdateCallback(PointCloudRuntime* runtime,
                                                   osgViewer::Viewer* viewer,
                                                   float* pointSize)
    : m_runtime(runtime)
    , m_viewer(viewer)
    , m_pointSize(pointSize)
{
}

void PointCloudUpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* visitor)
{
    if (m_runtime && m_viewer && m_pointSize) {
        m_runtime->update(m_viewer, *m_pointSize);
    }

    traverse(node, visitor);
}
