#pragma once

#include <osg/BoundingSphere>
#include <osg/Matrixd>
#include <osg/Vec3d>

namespace osg {
class Camera;
}

namespace osgGA {
class GUIEventAdapter;
}

namespace CameraMath
{
bool eventToFramebufferPixel(const osgGA::GUIEventAdapter& event,
                             const osg::Camera& camera,
                             int& pixelX,
                             int& pixelY);

bool projectWorldToFramebuffer(const osg::Vec3d& worldPoint,
                               const osg::Camera& camera,
                               osg::Vec3d& framebufferPoint);

bool unprojectFramebufferPoint(double pixelX,
                               double pixelY,
                               double depth,
                               const osg::Camera& camera,
                               osg::Vec3d& worldPoint);

bool buildPerspectiveMouseRay(const osgGA::GUIEventAdapter& event,
                              const osg::Camera& camera,
                              const osg::Vec3d& cameraEye,
                              osg::Vec3d& rayOrigin,
                              osg::Vec3d& rayDirection);

bool buildPerspectiveMouseRay(const osgGA::GUIEventAdapter& event,
                              const osg::Camera& camera,
                              const osg::Matrixd& viewMatrix,
                              const osg::Vec3d& cameraEye,
                              osg::Vec3d& rayOrigin,
                              osg::Vec3d& rayDirection);

bool intersectRayWithPlane(const osg::Vec3d& rayOrigin,
                           const osg::Vec3d& rayDirection,
                           const osg::Vec3d& planePoint,
                           const osg::Vec3d& planeNormal,
                           osg::Vec3d& intersection);

bool computeExponentialZoomDistance(double oldDistance,
                                    double wheelSteps,
                                    double zoomSensitivity,
                                    double minimumDistance,
                                    double maximumDistance,
                                    double& newDistance);

bool clampPitchDelta(const osg::Vec3d& cameraForward,
                     const osg::Vec3d& worldUp,
                     double requestedPitchDelta,
                     double minimumPitch,
                     double maximumPitch,
                     double& allowedPitchDelta);

bool computePerspectiveNearFar(const osg::Vec3d& cameraEye,
                               const osg::BoundingSphere& sceneBound,
                               double focusDistance,
                               double& nearPlane,
                               double& farPlane);
} // namespace CameraMath
