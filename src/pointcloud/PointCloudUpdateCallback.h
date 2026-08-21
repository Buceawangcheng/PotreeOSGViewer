#pragma once

#include <osg/NodeCallback>

#include <functional>

namespace osgViewer {
class Viewer;
}

class PointCloudRuntime;

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
