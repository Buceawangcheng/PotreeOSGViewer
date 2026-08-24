#pragma once

#include "viewer/camera/DepthBufferPicker.h"
#include "viewer/camera/PickDebugVisualizer.h"

#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/observer_ptr>
#include <osgGA/CameraManipulator>
#include <osgGA/GUIEventAdapter>

#include <functional>

namespace osg {
class Camera;
class Node;
}

class CesiumCameraManipulator : public osgGA::CameraManipulator
{
public:
    using PostViewUpdateCallback = std::function<void(osg::Camera&)>;

    CesiumCameraManipulator();

    const char* className() const override;

    void setNode(osg::Node* node) override;
    osg::Node* getNode() override;
    const osg::Node* getNode() const override;

    void setByMatrix(const osg::Matrixd& matrix) override;
    void setByInverseMatrix(const osg::Matrixd& matrix) override;
    osg::Matrixd getMatrix() const override;
    osg::Matrixd getInverseMatrix() const override;
    void updateCamera(osg::Camera& camera) override;
    bool handle(const osgGA::GUIEventAdapter& event,
                osgGA::GUIActionAdapter& action) override;

    void setPostViewUpdateCallback(PostViewUpdateCallback callback);
    void setMathDebugLoggingEnabled(bool enabled);
    void setDepthBufferPicker(DepthBufferPicker* picker);
    void setPickDebugVisualizer(PickDebugVisualizer* visualizer);
    void setPickDebugVisible(bool visible);
    bool pickDebugVisible() const;
    void setIgnoreHorizontalRotationInput(bool ignore);
    bool ignoreHorizontalRotationInput() const;
    void setIgnoreVerticalRotationInput(bool ignore);
    bool ignoreVerticalRotationInput() const;
    void setFocusDistance(double distance);
    double focusDistance() const;
    void invalidatePickRequests();

    void home(const osgGA::GUIEventAdapter& event,
              osgGA::GUIActionAdapter& action) override;
    void home(double currentTime) override;

protected:
    ~CesiumCameraManipulator() override = default;

private:
    enum class InteractionMode
    {
        None,
        PendingPan,
        Pan,
        PendingRotate,
        Rotate
    };

    struct CameraPose
    {
        osg::Vec3d eye;
        osg::Quat rotation;
        osg::Vec3d focusPoint;
    };

    bool computeHomePose(const osg::Camera* camera,
                         CameraPose& pose,
                         double& focusDistance) const;
    void applyHomePose(const osg::Camera* camera);
    bool buildMouseRay(const osgGA::GUIEventAdapter& event,
                       const osg::Camera& camera,
                       osg::Vec3d& rayOrigin,
                       osg::Vec3d& rayDirection) const;
    bool handlePush(const osgGA::GUIEventAdapter& event,
                    const osg::Camera& camera);
    bool handleDrag(const osgGA::GUIEventAdapter& event,
                    const osg::Camera& camera);
    bool handleRelease(const osgGA::GUIEventAdapter& event);
    bool handleScroll(const osgGA::GUIEventAdapter& event,
                      const osg::Camera& camera);
    bool queueDepthZoom(const osgGA::GUIEventAdapter& event,
                        const osg::Camera& camera);
    bool applyZoomFromRay(const osg::Vec3d& rayDirection, double wheelSteps);
    bool applyZoom(const osg::Vec3d& pivot, double wheelSteps);
    bool updatePan(const osgGA::GUIEventAdapter& event,
                   const osg::Camera& camera);
    bool updateRotate(double deltaX, double deltaY);
    bool requestDepthPick(const osgGA::GUIEventAdapter& event,
                          const osg::Camera& camera,
                          PickAction action,
                          double wheelSteps,
                          std::uint64_t& sequence);
    bool consumeDepthPickResult(const osg::Camera* camera);
    void resolvePendingGestureFallback();
    void invalidatePendingPickSequence();
    void logRotationPivot(const char* source) const;
    void logMouseRay(const osgGA::GUIEventAdapter& event,
                     const osg::Camera& camera) const;

    osg::observer_ptr<osg::Node> m_sceneNode;
    CameraPose m_pose;
    InteractionMode m_interactionMode = InteractionMode::None;
    osg::Vec3d m_worldUp = osg::Vec3d(0.0, 0.0, 1.0);
    osg::Vec3d m_panPlanePoint;
    osg::Vec3d m_panPlaneNormal;
    osg::Vec3d m_rotationPivot;
    osg::Vec3d m_pressForward;
    osg::ref_ptr<osgGA::GUIEventAdapter> m_latestGestureEvent;
    double m_lastMouseX = 0.0;
    double m_lastMouseY = 0.0;
    double m_focusDistance = 10.0;
    double m_minFocusDistance = 0.01;
    double m_maxFocusDistance = 1000000.0;
    double m_zoomSensitivity = 0.15;
    double m_rotationSensitivity = 0.003;
    osg::ref_ptr<DepthBufferPicker> m_depthBufferPicker;
    osg::ref_ptr<PickDebugVisualizer> m_pickDebugVisualizer;
    std::uint64_t m_pickGeneration = 1;
    std::uint64_t m_nextPickSequence = 1;
    std::uint64_t m_latestPickSequence = 0;
    std::uint64_t m_pendingGestureSequence = 0;
    std::uint64_t m_pendingZoomSequence = 0;
    double m_pendingZoomWheelSteps = 0.0;
    osg::Vec3d m_pendingZoomRayDirection;
    bool m_pickRequestOutstanding = false;
    bool m_pickDebugVisible = false;
    bool m_ignoreHorizontalRotationInput = false;
    bool m_ignoreVerticalRotationInput = false;
    PostViewUpdateCallback m_postViewUpdateCallback;
    bool m_mathDebugLoggingEnabled = false;
};
