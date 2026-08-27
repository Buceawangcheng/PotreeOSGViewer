#pragma once

#include <osg/NodeCallback>

#include <functional>

namespace osgViewer {
class Viewer;
}

class PointCloudRuntime;

// 挂在场景根节点上的 OSG update traversal 回调。
// Viewer 保持 SingleThreaded，因此 operator()、PointCloudRuntime::update()、
// Octree 修改和 OSG 节点挂载都在同一主/update线程中执行。
class PointCloudUpdateCallback : public osg::NodeCallback {
public:
    PointCloudUpdateCallback(PointCloudRuntime* runtime,
                             osgViewer::Viewer* viewer,
                             float* pointSize,
                             std::function<void()> frameCallback = {});

    void operator()(osg::Node* node, osg::NodeVisitor* visitor) override;

private:
    PointCloudRuntime* m_runtime = nullptr;
    osgViewer::Viewer* m_viewer = nullptr;
    float* m_pointSize = nullptr;
    std::function<void()> m_frameCallback;
};
