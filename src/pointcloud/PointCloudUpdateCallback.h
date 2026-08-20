#pragma once

#include <osg/NodeCallback>

namespace osgViewer {
class Viewer;
}

class PointCloudRuntime;

class PointCloudUpdateCallback : public osg::NodeCallback {
public:
    PointCloudUpdateCallback(PointCloudRuntime* runtime,
                             osgViewer::Viewer* viewer,
                             float* pointSize);

    void operator()(osg::Node* node, osg::NodeVisitor* visitor) override;

private:
    PointCloudRuntime* m_runtime = nullptr;
    osgViewer::Viewer* m_viewer = nullptr;
    float* m_pointSize = nullptr;
};
