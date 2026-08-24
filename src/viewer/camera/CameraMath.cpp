#include "viewer/camera/CameraMath.h"

#include <osg/BoundingSphere>
#include <osg/Camera>
#include <osg/Matrixd>
#include <osg/Viewport>
#include <osgGA/GUIEventAdapter>

#include <algorithm>
#include <cmath>

namespace
{
bool finiteVec(const osg::Vec3d& value)
{
    return std::isfinite(value.x())
        && std::isfinite(value.y())
        && std::isfinite(value.z());
}

bool validViewport(const osg::Viewport* viewport)
{
    return viewport
        && std::isfinite(viewport->x())
        && std::isfinite(viewport->y())
        && std::isfinite(viewport->width())
        && std::isfinite(viewport->height())
        && viewport->width() > 0.0
        && viewport->height() > 0.0;
}

bool worldWindowMatrices(const osg::Matrixd& viewMatrix,
                         const osg::Matrixd& projectionMatrix,
                         const osg::Viewport* viewport,
                         osg::Matrixd& worldToWindow,
                         osg::Matrixd& windowToWorld)
{
    if (!validViewport(viewport)) {
        return false;
    }

    worldToWindow = viewMatrix
        * projectionMatrix
        * viewport->computeWindowMatrix();
    return windowToWorld.invert(worldToWindow);
}

bool worldWindowMatrices(const osg::Camera& camera,
                         osg::Matrixd& worldToWindow,
                         osg::Matrixd& windowToWorld)
{
    return worldWindowMatrices(camera.getViewMatrix(),
                               camera.getProjectionMatrix(),
                               camera.getViewport(),
                               worldToWindow,
                               windowToWorld);
}

bool unprojectWithViewMatrix(double pixelX,
                             double pixelY,
                             double depth,
                             const osg::Camera& camera,
                             const osg::Matrixd& viewMatrix,
                             osg::Vec3d& worldPoint)
{
    if (!std::isfinite(pixelX) || !std::isfinite(pixelY)
        || !std::isfinite(depth) || depth < 0.0 || depth > 1.0) {
        return false;
    }

    osg::Matrixd worldToWindow;
    osg::Matrixd windowToWorld;
    if (!worldWindowMatrices(viewMatrix,
                             camera.getProjectionMatrix(),
                             camera.getViewport(),
                             worldToWindow,
                             windowToWorld)) {
        return false;
    }

    const osg::Vec3d unprojected = osg::Vec3d(pixelX, pixelY, depth)
        * windowToWorld;
    if (!finiteVec(unprojected)) {
        return false;
    }

    worldPoint = unprojected;
    return true;
}
} // namespace

namespace CameraMath
{
bool eventToFramebufferPixel(const osgGA::GUIEventAdapter& event,
                             const osg::Camera& camera,
                             int& pixelX,
                             int& pixelY)
{
    const osg::Viewport* viewport = camera.getViewport();
    if (!validViewport(viewport)
        || !std::isfinite(event.getX())
        || !std::isfinite(event.getY())) {
        return false;
    }

    // osgQOpenGL has already converted Qt logical coordinates to physical pixels.
    // osgViewer normally normalizes pointer events to Y_INCREASING_UPWARDS before
    // dispatch. Honor the event metadata as other injection paths may retain the
    // original top-to-bottom orientation.
    const double framebufferX = std::floor(static_cast<double>(event.getX()));
    double eventY = static_cast<double>(event.getY());
    if (event.getMouseYOrientation()
        == osgGA::GUIEventAdapter::Y_INCREASING_DOWNWARDS) {
        eventY = static_cast<double>(event.getYmax())
            - eventY
            + static_cast<double>(event.getYmin());
    }
    const double framebufferY = std::floor(eventY);

    if (framebufferX < viewport->x()
        || framebufferX >= viewport->x() + viewport->width()
        || framebufferY < viewport->y()
        || framebufferY >= viewport->y() + viewport->height()) {
        return false;
    }

    pixelX = static_cast<int>(framebufferX);
    pixelY = static_cast<int>(framebufferY);
    return true;
}

bool projectWorldToFramebuffer(const osg::Vec3d& worldPoint,
                               const osg::Camera& camera,
                               osg::Vec3d& framebufferPoint)
{
    if (!finiteVec(worldPoint)) {
        return false;
    }

    osg::Matrixd worldToWindow;
    osg::Matrixd windowToWorld;
    if (!worldWindowMatrices(camera, worldToWindow, windowToWorld)) {
        return false;
    }

    const osg::Vec3d projected = worldPoint * worldToWindow;
    if (!finiteVec(projected)) {
        return false;
    }

    framebufferPoint = projected;
    return true;
}

bool unprojectFramebufferPoint(double pixelX,
                               double pixelY,
                               double depth,
                               const osg::Camera& camera,
                               osg::Vec3d& worldPoint)
{
    if (!std::isfinite(pixelX) || !std::isfinite(pixelY)
        || !std::isfinite(depth) || depth < 0.0 || depth > 1.0) {
        return false;
    }

    osg::Matrixd worldToWindow;
    osg::Matrixd windowToWorld;
    if (!worldWindowMatrices(camera, worldToWindow, windowToWorld)) {
        return false;
    }

    const osg::Vec3d unprojected = osg::Vec3d(pixelX, pixelY, depth)
        * windowToWorld;
    if (!finiteVec(unprojected)) {
        return false;
    }

    worldPoint = unprojected;
    return true;
}

bool buildPerspectiveMouseRay(const osgGA::GUIEventAdapter& event,
                              const osg::Camera& camera,
                              const osg::Vec3d& cameraEye,
                              osg::Vec3d& rayOrigin,
                              osg::Vec3d& rayDirection)
{
    return buildPerspectiveMouseRay(event,
                                    camera,
                                    camera.getViewMatrix(),
                                    cameraEye,
                                    rayOrigin,
                                    rayDirection);
}

bool buildPerspectiveMouseRay(const osgGA::GUIEventAdapter& event,
                              const osg::Camera& camera,
                              const osg::Matrixd& viewMatrix,
                              const osg::Vec3d& cameraEye,
                              osg::Vec3d& rayOrigin,
                              osg::Vec3d& rayDirection)
{
    if (!finiteVec(cameraEye)) {
        return false;
    }

    int pixelX = 0;
    int pixelY = 0;
    if (!eventToFramebufferPixel(event, camera, pixelX, pixelY)) {
        return false;
    }

    osg::Vec3d nearPoint;
    osg::Vec3d farPoint;
    if (!unprojectWithViewMatrix(
            pixelX, pixelY, 0.0, camera, viewMatrix, nearPoint)
        || !unprojectWithViewMatrix(
            pixelX, pixelY, 1.0, camera, viewMatrix, farPoint)) {
        return false;
    }

    osg::Vec3d direction = farPoint - nearPoint;
    const double length = direction.length();
    if (!std::isfinite(length) || length <= 1.0e-12) {
        return false;
    }
    direction /= length;

    rayOrigin = cameraEye;
    rayDirection = direction;
    return true;
}

bool intersectRayWithPlane(const osg::Vec3d& rayOrigin,
                           const osg::Vec3d& rayDirection,
                           const osg::Vec3d& planePoint,
                           const osg::Vec3d& planeNormal,
                           osg::Vec3d& intersection)
{
    if (!finiteVec(rayOrigin) || !finiteVec(rayDirection)
        || !finiteVec(planePoint) || !finiteVec(planeNormal)
        || rayDirection.length2() <= 1.0e-24
        || planeNormal.length2() <= 1.0e-24) {
        return false;
    }

    const double denominator = rayDirection * planeNormal;
    if (!std::isfinite(denominator) || std::abs(denominator) <= 1.0e-12) {
        return false;
    }

    const double rayParameter = ((planePoint - rayOrigin) * planeNormal)
        / denominator;
    if (!std::isfinite(rayParameter) || rayParameter < 0.0) {
        return false;
    }

    const osg::Vec3d result = rayOrigin + rayDirection * rayParameter;
    if (!finiteVec(result)) {
        return false;
    }

    intersection = result;
    return true;
}

bool computeExponentialZoomDistance(double oldDistance,
                                    double wheelSteps,
                                    double zoomSensitivity,
                                    double minimumDistance,
                                    double maximumDistance,
                                    double& newDistance)
{
    if (!std::isfinite(oldDistance) || oldDistance <= 1.0e-12
        || !std::isfinite(wheelSteps)
        || !std::isfinite(zoomSensitivity)
        || zoomSensitivity <= 0.0 || zoomSensitivity >= 1.0
        || !std::isfinite(minimumDistance) || minimumDistance <= 0.0
        || !std::isfinite(maximumDistance)
        || maximumDistance < minimumDistance) {
        return false;
    }

    const double requestedScale = std::pow(1.0 - zoomSensitivity, wheelSteps);
    if (!std::isfinite(requestedScale) || requestedScale <= 0.0) {
        return false;
    }

    const double requestedDistance = oldDistance * requestedScale;
    if (!std::isfinite(requestedDistance)) {
        return false;
    }

    newDistance = std::clamp(
        requestedDistance, minimumDistance, maximumDistance);
    return true;
}

bool clampPitchDelta(const osg::Vec3d& cameraForward,
                     const osg::Vec3d& worldUp,
                     double requestedPitchDelta,
                     double minimumPitch,
                     double maximumPitch,
                     double& allowedPitchDelta)
{
    if (!finiteVec(cameraForward) || cameraForward.length2() <= 1.0e-24
        || !finiteVec(worldUp) || worldUp.length2() <= 1.0e-24
        || !std::isfinite(requestedPitchDelta)
        || !std::isfinite(minimumPitch) || !std::isfinite(maximumPitch)
        || maximumPitch < minimumPitch) {
        return false;
    }

    const osg::Vec3d normalizedForward = cameraForward
        / cameraForward.length();
    const osg::Vec3d normalizedWorldUp = worldUp / worldUp.length();
    const double forwardUpDot = std::clamp(
        normalizedForward * normalizedWorldUp, -1.0, 1.0);
    const double currentPitch = std::asin(forwardUpDot);
    const double desiredPitch = std::clamp(
        currentPitch + requestedPitchDelta, minimumPitch, maximumPitch);
    const double result = desiredPitch - currentPitch;
    if (!std::isfinite(result)) {
        return false;
    }

    allowedPitchDelta = result;
    return true;
}

bool computePerspectiveNearFar(const osg::Vec3d& cameraEye,
                               const osg::BoundingSphere& sceneBound,
                               double focusDistance,
                               double& nearPlane,
                               double& farPlane)
{
    if (!finiteVec(cameraEye) || !sceneBound.valid()
        || !finiteVec(sceneBound.center())
        || !std::isfinite(sceneBound.radius())
        || !std::isfinite(focusDistance) || focusDistance <= 0.0) {
        return false;
    }

    const double radius = std::max(static_cast<double>(sceneBound.radius()), 1.0);
    const double distance = (cameraEye - sceneBound.center()).length();
    if (!std::isfinite(distance)) {
        return false;
    }

    const double padding = std::max(radius * 0.1, 1.0e-6);
    // Keep the near plane well in front of the eye while tying its scale to
    // the current interaction distance. A radius-only epsilon becomes far too
    // small once the eye moves inside a large scene bound and collapses useful
    // 24-bit depth precision near the focus surface.
    const double minimumNear = std::max(focusDistance * 1.0e-3, 1.0e-6);
    const double computedNear = std::max(
        minimumNear, distance - radius - padding);
    const double computedFar = std::max(
        computedNear * 1.01, distance + radius + padding);

    if (!std::isfinite(computedNear) || !std::isfinite(computedFar)
        || computedNear <= 0.0 || computedFar <= computedNear) {
        return false;
    }

    nearPlane = computedNear;
    farPlane = computedFar;
    return true;
}
} // namespace CameraMath
