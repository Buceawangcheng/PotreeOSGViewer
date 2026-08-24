#include "viewer/camera/CesiumCameraManipulator.h"

#include "viewer/camera/CameraMath.h"

#include <osg/BoundingSphere>
#include <osg/Camera>
#include <osg/Math>
#include <osg/Node>
#include <osg/Notify>
#include <osg/View>
#include <osg/Viewport>

#include <QDebug>
#include <QString>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <QDebug>

namespace
{
constexpr double MinimumHomeRadius = 1.0e-6;
constexpr double DefaultHomeDistanceScale = 4.0;
constexpr double HomeFramingPadding = 1.05;
constexpr double PitchLimitRadians =
    89.0 * 3.14159265358979323846 / 180.0;
constexpr double MinimumPitch = -PitchLimitRadians;
constexpr double MaximumPitch = PitchLimitRadians;

bool finiteVec(const osg::Vec3d& value)
{
    return std::isfinite(value.x())
        && std::isfinite(value.y())
        && std::isfinite(value.z());
}

bool finiteQuat(const osg::Quat& value)
{
    return std::isfinite(value.x())
        && std::isfinite(value.y())
        && std::isfinite(value.z())
        && std::isfinite(value.w());
}

const char* depthPickOutcomeName(DepthPickOutcome outcome)
{
    switch (outcome) {
    case DepthPickOutcome::Hit:
        return "Hit";
    case DepthPickOutcome::MissingCameraOrViewport:
        return "MissingCameraOrViewport";
    case DepthPickOutcome::InvalidReadRegion:
        return "InvalidReadRegion";
    case DepthPickOutcome::ReadPixelsError:
        return "ReadPixelsError";
    case DepthPickOutcome::NoValidDepth:
        return "NoValidDepth";
    case DepthPickOutcome::UnprojectionFailed:
        return "UnprojectionFailed";
    case DepthPickOutcome::Unknown:
        return "Unknown";
    }
    return "Unknown";
}

bool rotationFromLookAt(const osg::Vec3d& eye,
                        const osg::Vec3d& center,
                        const osg::Vec3d& up,
                        osg::Quat& rotation)
{
    if (!finiteVec(eye) || !finiteVec(center) || !finiteVec(up)
        || (center - eye).length2() <= 0.0 || up.length2() <= 0.0) {
        return false;
    }

    osg::Matrixd viewMatrix;
    viewMatrix.makeLookAt(eye, center, up);
    osg::Matrixd cameraMatrix;
    if (!cameraMatrix.invert(viewMatrix)) {
        return false;
    }

    rotation = cameraMatrix.getRotate();
    if (!finiteQuat(rotation) || rotation.length2() <= 0.0) {
        return false;
    }
    rotation /= rotation.length();

    return true;
}
} // namespace

CesiumCameraManipulator::CesiumCameraManipulator()
{
    m_pose.eye.set(0.0, -10.0, 0.0);
    m_pose.focusPoint.set(0.0, 0.0, 0.0);
    rotationFromLookAt(m_pose.eye,
                       m_pose.focusPoint,
                       osg::Vec3d(0.0, 0.0, 1.0),
                       m_pose.rotation);
}

const char* CesiumCameraManipulator::className() const
{
    return "CesiumCameraManipulator";
}

void CesiumCameraManipulator::setNode(osg::Node* node)
{
    invalidatePickRequests();
    m_sceneNode = node;
}

osg::Node* CesiumCameraManipulator::getNode()
{
    return m_sceneNode.get();
}

const osg::Node* CesiumCameraManipulator::getNode() const
{
    return m_sceneNode.get();
}

void CesiumCameraManipulator::setFocusDistance(double distance)
{
    if (!std::isfinite(distance) || distance <= 0.0) {
        return;
    }

    m_focusDistance = std::clamp(
        distance, m_minFocusDistance, m_maxFocusDistance);
    m_pose.focusPoint = m_pose.eye
        + m_pose.rotation * osg::Vec3d(0.0, 0.0, -m_focusDistance);
}

double CesiumCameraManipulator::focusDistance() const
{
    return m_focusDistance;
}

void CesiumCameraManipulator::setByMatrix(const osg::Matrixd& matrix)
{
    const osg::Vec3d eye = matrix.getTrans();
    osg::Quat rotation = matrix.getRotate();
    if (!finiteVec(eye) || !finiteQuat(rotation) || rotation.length2() <= 0.0) {
        return;
    }

    if (m_depthBufferPicker.valid()) {
        invalidatePendingPickSequence();
    }

    rotation /= rotation.length();
    const double focusDistance = std::clamp(
        std::isfinite(m_focusDistance) && m_focusDistance > 0.0
            ? m_focusDistance
            : 1.0,
        m_minFocusDistance,
        m_maxFocusDistance);

    m_pose.eye = eye;
    m_pose.rotation = rotation;
    m_pose.focusPoint = eye + rotation * osg::Vec3d(0.0, 0.0, -focusDistance);
    m_focusDistance = focusDistance;
    m_interactionMode = InteractionMode::None;
}

void CesiumCameraManipulator::setByInverseMatrix(const osg::Matrixd& matrix)
{
    osg::Matrixd cameraMatrix;
    if (cameraMatrix.invert(matrix)) {
        setByMatrix(cameraMatrix);
    }
}

osg::Matrixd CesiumCameraManipulator::getMatrix() const
{
    return osg::Matrixd::rotate(m_pose.rotation)
        * osg::Matrixd::translate(m_pose.eye);
}

osg::Matrixd CesiumCameraManipulator::getInverseMatrix() const
{
    return osg::Matrixd::translate(-m_pose.eye)
        * osg::Matrixd::rotate(m_pose.rotation.inverse());
}

void CesiumCameraManipulator::updateCamera(osg::Camera& camera)
{
    osgGA::CameraManipulator::updateCamera(camera);
    if (m_postViewUpdateCallback) {
        m_postViewUpdateCallback(camera);
    }
}

bool CesiumCameraManipulator::handle(const osgGA::GUIEventAdapter& event,
                                     osgGA::GUIActionAdapter& action)
{
    const osg::View* view = action.asView();
    const osg::Camera* camera = view ? view->getCamera() : nullptr;
    const bool moveEvent = event.getEventType()
        == osgGA::GUIEventAdapter::MOVE;
    if (moveEvent) {
        if (m_mathDebugLoggingEnabled && camera) {
            logMouseRay(event, *camera);
        }
    }

    bool handled = false;
    switch (moveEvent ? osgGA::GUIEventAdapter::NONE : event.getEventType()) {
    case osgGA::GUIEventAdapter::PUSH:
        handled = camera && handlePush(event, *camera);
        if (handled && camera
            && m_interactionMode == InteractionMode::PendingPan) {
            if (!requestDepthPick(event,
                                  *camera,
                                  PickAction::BeginPan,
                                  0.0,
                                  m_pendingGestureSequence)) {
                resolvePendingGestureFallback();
            }
        } else if (handled && camera
                   && m_interactionMode == InteractionMode::PendingRotate) {
            if (!requestDepthPick(event,
                                  *camera,
                                  PickAction::BeginRotate,
                                  0.0,
                                  m_pendingGestureSequence)) {
                resolvePendingGestureFallback();
            }
        }
        break;
    case osgGA::GUIEventAdapter::DRAG:
        handled = camera && handleDrag(event, *camera);
        break;
    case osgGA::GUIEventAdapter::RELEASE:
        handled = handleRelease(event);
        break;
    case osgGA::GUIEventAdapter::SCROLL:
        handled = camera && (m_depthBufferPicker.valid()
            ? queueDepthZoom(event, *camera)
            : handleScroll(event, *camera));
        break;
    default:
        break;
    }

    // Register a newer request before consuming the previous frame's mailbox.
    // Exact generation/sequence matching then prevents a superseded marker from
    // becoming visible for one frame.
    const bool consumedPickResult = consumeDepthPickResult(camera);

    if (handled || consumedPickResult) {
        action.requestRedraw();
    }
    if (m_pickRequestOutstanding) {
        action.requestContinuousUpdate(true);
    } else if (handled || consumedPickResult) {
        action.requestContinuousUpdate(false);
    }
    return handled;
}

bool CesiumCameraManipulator::buildMouseRay(
    const osgGA::GUIEventAdapter& event,
    const osg::Camera& camera,
    osg::Vec3d& rayOrigin,
    osg::Vec3d& rayDirection) const
{
    return CameraMath::buildPerspectiveMouseRay(
        event,
        camera,
        getInverseMatrix(),
        m_pose.eye,
        rayOrigin,
        rayDirection);
}

bool CesiumCameraManipulator::handlePush(
    const osgGA::GUIEventAdapter& event,
    const osg::Camera& camera)
{
    if (!std::isfinite(event.getX()) || !std::isfinite(event.getY())) {
        return false;
    }

    if (event.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        osg::Vec3d forward = m_pose.rotation * osg::Vec3d(0.0, 0.0, -1.0);
        const double forwardLength = forward.length();
        if (!finiteVec(forward) || !std::isfinite(forwardLength)
            || forwardLength <= 1.0e-12) {
            return false;
        }
        forward /= forwardLength;

        osg::Vec3d rayOrigin;
        osg::Vec3d rayDirection;
        osg::Vec3d pressIntersection;
        if (!buildMouseRay(event, camera, rayOrigin, rayDirection)
            || !CameraMath::intersectRayWithPlane(
                rayOrigin,
                rayDirection,
                m_pose.focusPoint,
                forward,
                pressIntersection)) {
            return false;
        }

        if (m_depthBufferPicker.valid()) {
            invalidatePendingPickSequence();
            m_pressForward = forward;
            m_latestGestureEvent = new osgGA::GUIEventAdapter(event);
            m_interactionMode = InteractionMode::PendingPan;
        } else {
            m_panPlanePoint = m_pose.focusPoint;
            m_panPlaneNormal = forward;
            m_interactionMode = InteractionMode::Pan;
        }
        m_lastMouseX = event.getX();
        m_lastMouseY = event.getY();
        return true;
    }

    if (event.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) {
        if (m_depthBufferPicker.valid()) {
            invalidatePendingPickSequence();
            m_latestGestureEvent = new osgGA::GUIEventAdapter(event);
            m_interactionMode = InteractionMode::PendingRotate;
        } else {
            m_rotationPivot = m_pose.focusPoint;
            logRotationPivot("DefaultFocusPoint");
            m_interactionMode = InteractionMode::Rotate;
        }
        m_lastMouseX = event.getX();
        m_lastMouseY = event.getY();
        return true;
    }

    return false;
}

bool CesiumCameraManipulator::handleDrag(
    const osgGA::GUIEventAdapter& event,
    const osg::Camera& camera)
{
    if (!std::isfinite(event.getX()) || !std::isfinite(event.getY())) {
        return false;
    }

    if (m_interactionMode == InteractionMode::PendingPan) {
        if ((event.getButtonMask()
             & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) == 0) {
            invalidatePendingPickSequence();
            m_interactionMode = InteractionMode::None;
            m_latestGestureEvent = nullptr;
            return false;
        }
        m_latestGestureEvent = new osgGA::GUIEventAdapter(event);
        return true;
    }

    if (m_interactionMode == InteractionMode::Pan) {
        if ((event.getButtonMask()
             & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) == 0) {
            m_interactionMode = InteractionMode::None;
            return false;
        }
        return updatePan(event, camera);
    }

    if (m_interactionMode == InteractionMode::PendingRotate) {
        if ((event.getButtonMask()
             & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) == 0) {
            invalidatePendingPickSequence();
            m_interactionMode = InteractionMode::None;
            m_latestGestureEvent = nullptr;
            return false;
        }
        m_latestGestureEvent = new osgGA::GUIEventAdapter(event);
        return true;
    }

    if (m_interactionMode == InteractionMode::Rotate) {
        if ((event.getButtonMask()
             & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON) == 0) {
            m_interactionMode = InteractionMode::None;
            return false;
        }
        const double deltaX = static_cast<double>(event.getX()) - m_lastMouseX;
        const double deltaY = static_cast<double>(event.getY()) - m_lastMouseY;
        m_lastMouseX = event.getX();
        m_lastMouseY = event.getY();
        return updateRotate(deltaX, deltaY);
    }

    return false;
}

bool CesiumCameraManipulator::handleRelease(
    const osgGA::GUIEventAdapter& event)
{
    const bool releasesPendingPan =
        m_interactionMode == InteractionMode::PendingPan
        && event.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
    const bool releasesPan = m_interactionMode == InteractionMode::Pan
        && event.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
    const bool releasesPendingRotate =
        m_interactionMode == InteractionMode::PendingRotate
        && event.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
    const bool releasesRotate = m_interactionMode == InteractionMode::Rotate
        && event.getButton() == osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
    if (!releasesPendingPan && !releasesPan
        && !releasesPendingRotate && !releasesRotate) {
        return false;
    }

    if (releasesPendingPan || releasesPendingRotate) {
        invalidatePendingPickSequence();
    }
    m_interactionMode = InteractionMode::None;
    m_latestGestureEvent = nullptr;
    return true;
}

bool CesiumCameraManipulator::handleScroll(
    const osgGA::GUIEventAdapter& event,
    const osg::Camera& camera)
{
    if (m_interactionMode != InteractionMode::None) {
        return false;
    }

    double wheelSteps = 0.0;
    if (event.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP) {
        wheelSteps = 1.0;
    } else if (event.getScrollingMotion()
               == osgGA::GUIEventAdapter::SCROLL_DOWN) {
        wheelSteps = -1.0;
    } else {
        return false;
    }

    osg::Vec3d rayOrigin;
    osg::Vec3d rayDirection;
    return buildMouseRay(event, camera, rayOrigin, rayDirection)
        && applyZoomFromRay(rayDirection, wheelSteps);
}

bool CesiumCameraManipulator::queueDepthZoom(
    const osgGA::GUIEventAdapter& event,
    const osg::Camera& camera)
{
    if (m_interactionMode != InteractionMode::None) {
        return false;
    }

    double wheelSteps = 0.0;
    if (event.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP) {
        wheelSteps = 1.0;
    } else if (event.getScrollingMotion()
               == osgGA::GUIEventAdapter::SCROLL_DOWN) {
        wheelSteps = -1.0;
    } else {
        return false;
    }

    osg::Vec3d rayOrigin;
    osg::Vec3d rayDirection;
    if (!buildMouseRay(event, camera, rayOrigin, rayDirection)
        || !finiteVec(rayDirection) || rayDirection.length2() <= 1.0e-24) {
        return false;
    }

    m_pendingZoomWheelSteps += wheelSteps;
    m_pendingZoomRayDirection = rayDirection / rayDirection.length();
    std::uint64_t sequence = 0;
    if (!requestDepthPick(event,
                          camera,
                          PickAction::Zoom,
                          m_pendingZoomWheelSteps,
                          sequence)) {
        const double fallbackSteps = m_pendingZoomWheelSteps;
        m_pendingZoomWheelSteps = 0.0;
        m_pendingZoomSequence = 0;
        return applyZoomFromRay(m_pendingZoomRayDirection, fallbackSteps);
    }

    m_pendingZoomSequence = sequence;
    return true;
}

bool CesiumCameraManipulator::applyZoomFromRay(
    const osg::Vec3d& rayDirection,
    double wheelSteps)
{
    if (!finiteVec(rayDirection) || rayDirection.length2() <= 1.0e-24
        || !std::isfinite(m_focusDistance)) {
        return false;
    }

    osg::Vec3d normalizedRayDirection = rayDirection / rayDirection.length();
    const osg::Vec3d zoomPivot = m_pose.eye
        + normalizedRayDirection * m_focusDistance;
    return applyZoom(zoomPivot, wheelSteps);
}

bool CesiumCameraManipulator::applyZoom(const osg::Vec3d& zoomPivot,
                                        double wheelSteps)
{
    if (!finiteVec(zoomPivot) || !std::isfinite(wheelSteps)) {
        return false;
    }

    const osg::Vec3d oldEye = m_pose.eye;
    const double oldDistance = (oldEye - zoomPivot).length();
    double newDistance = 0.0;
    if (!CameraMath::computeExponentialZoomDistance(
            oldDistance,
            wheelSteps,
            m_zoomSensitivity,
            m_minFocusDistance,
            m_maxFocusDistance,
            newDistance)) {
        return false;
    }

    const double effectiveScale = newDistance / oldDistance;
    const osg::Vec3d newEye = zoomPivot
        + (oldEye - zoomPivot) * effectiveScale;
    const osg::Vec3d translation = newEye - oldEye;
    if (!finiteVec(newEye) || !finiteVec(translation)) {
        return false;
    }

    m_pose.eye = newEye;
    m_pose.focusPoint += translation;
    m_focusDistance = newDistance;
    return true;
}

bool CesiumCameraManipulator::updatePan(
    const osgGA::GUIEventAdapter& event,
    const osg::Camera& camera)
{
    osg::ref_ptr<osgGA::GUIEventAdapter> previousEvent =
        new osgGA::GUIEventAdapter(event);
    previousEvent->setX(static_cast<float>(m_lastMouseX));
    previousEvent->setY(static_cast<float>(m_lastMouseY));

    osg::Vec3d previousRayOrigin;
    osg::Vec3d previousRayDirection;
    osg::Vec3d currentRayOrigin;
    osg::Vec3d currentRayDirection;
    osg::Vec3d previousWorldPoint;
    osg::Vec3d currentWorldPoint;
    if (!buildMouseRay(
            *previousEvent,
            camera,
            previousRayOrigin,
            previousRayDirection)
        || !buildMouseRay(
            event,
            camera,
            currentRayOrigin,
            currentRayDirection)
        || !CameraMath::intersectRayWithPlane(
            previousRayOrigin,
            previousRayDirection,
            m_panPlanePoint,
            m_panPlaneNormal,
            previousWorldPoint)
        || !CameraMath::intersectRayWithPlane(
            currentRayOrigin,
            currentRayDirection,
            m_panPlanePoint,
            m_panPlaneNormal,
            currentWorldPoint)) {
        return false;
    }

    const osg::Vec3d translation = previousWorldPoint - currentWorldPoint;
    if (!finiteVec(translation)) {
        return false;
    }

    m_pose.eye += translation;
    m_pose.focusPoint += translation;
    m_lastMouseX = event.getX();
    m_lastMouseY = event.getY();
    return true;
}

bool CesiumCameraManipulator::updateRotate(double deltaX, double deltaY)
{
    const double rawDeltaX = deltaX;
    const double rawDeltaY = deltaY;
    if (m_ignoreHorizontalRotationInput) {
        deltaX = 0.0;
    }
    if (m_ignoreVerticalRotationInput) {
        deltaY = 0.0;
    }

    if (!std::isfinite(deltaX) || !std::isfinite(deltaY)
        || !finiteVec(m_rotationPivot) || !finiteVec(m_worldUp)
        || m_worldUp.length2() <= 1.0e-24) {
        return false;
    }
    osg::Vec3d normalizedWorldUp = m_worldUp / m_worldUp.length();
    const double yawAngle = -deltaX * m_rotationSensitivity;
    const double requestedPitchDelta = -deltaY * m_rotationSensitivity;

    osg::Vec3d cameraForward = m_pose.rotation
        * osg::Vec3d(0.0, 0.0, -1.0);
    const double eyeZBefore = m_pose.eye.z();
    const double forwardZBefore = cameraForward.z();
    double allowedPitchDelta = 0.0;
    if (!CameraMath::clampPitchDelta(
            cameraForward,
            normalizedWorldUp,
            requestedPitchDelta,
            MinimumPitch,
            MaximumPitch,
            allowedPitchDelta)) {
        return false;
    }

    const osg::Quat yawRotation(yawAngle, normalizedWorldUp);
    osg::Quat yawedRotation = yawRotation * m_pose.rotation;
    osg::Vec3d yawedCameraRight = yawedRotation
        * osg::Vec3d(1.0, 0.0, 0.0);
    const double rightLength = yawedCameraRight.length();
    if (!finiteVec(yawedCameraRight) || !std::isfinite(rightLength)
        || rightLength <= 1.0e-12) {
        return false;
    }
    yawedCameraRight /= rightLength;

    const osg::Quat pitchRotation(allowedPitchDelta, yawedCameraRight);
    const osg::Quat deltaRotation = pitchRotation * yawRotation;
    const osg::Vec3d newEye = m_rotationPivot
        + deltaRotation * (m_pose.eye - m_rotationPivot);
    const osg::Vec3d newFocusPoint = m_rotationPivot
        + deltaRotation * (m_pose.focusPoint - m_rotationPivot);
    osg::Vec3d newForward = deltaRotation * cameraForward;
    const double forwardLength = newForward.length();
    if (!finiteVec(newEye) || !finiteVec(newFocusPoint)
        || !finiteVec(newForward) || !std::isfinite(forwardLength)
        || forwardLength <= 1.0e-12) {
        return false;
    }
    newForward /= forwardLength;

    osg::Vec3d correctedRight = newForward ^ normalizedWorldUp;
    const double correctedRightLength = correctedRight.length();
    if (!std::isfinite(correctedRightLength)
        || correctedRightLength <= 1.0e-12) {
        return false;
    }
    correctedRight /= correctedRightLength;
    osg::Vec3d correctedUp = correctedRight ^ newForward;
    correctedUp.normalize();

    osg::Quat correctedRotation;
    if (!rotationFromLookAt(
            newEye, newEye + newForward, correctedUp, correctedRotation)) {
        return false;
    }

    m_pose.eye = newEye;
    m_pose.focusPoint = newFocusPoint;
    m_pose.rotation = correctedRotation;
    /*qInfo().noquote() << QStringLiteral(
        "[CameraRotate] raw=(%1,%2) filtered=(%3,%4) ignoreH=%5 ignoreV=%6 "
        "eyeZ=%7->%8 forwardZ=%9->%10 pivot=(%11,%12,%13)")
        .arg(rawDeltaX, 0, 'g', 17)
        .arg(rawDeltaY, 0, 'g', 17)
        .arg(deltaX, 0, 'g', 17)
        .arg(deltaY, 0, 'g', 17)
        .arg(m_ignoreHorizontalRotationInput ? 1 : 0)
        .arg(m_ignoreVerticalRotationInput ? 1 : 0)
        .arg(eyeZBefore, 0, 'g', 17)
        .arg(newEye.z(), 0, 'g', 17)
        .arg(forwardZBefore, 0, 'g', 17)
        .arg(newForward.z(), 0, 'g', 17)
        .arg(m_rotationPivot.x(), 0, 'g', 17)
        .arg(m_rotationPivot.y(), 0, 'g', 17)
        .arg(m_rotationPivot.z(), 0, 'g', 17);*/
    return true;
}

void CesiumCameraManipulator::logMouseRay(
    const osgGA::GUIEventAdapter& event,
    const osg::Camera& camera) const
{
    int pixelX = 0;
    int pixelY = 0;
    osg::Vec3d rayOrigin;
    osg::Vec3d rayDirection;
    if (!CameraMath::eventToFramebufferPixel(event, camera, pixelX, pixelY)
        || !buildMouseRay(event, camera, rayOrigin, rayDirection)) {
        osg::notify(osg::NOTICE)
            << "[CameraMath] event outside viewport or ray construction failed: event=("
            << event.getX() << ", " << event.getY() << ") yOrientation="
            << (event.getMouseYOrientation()
                    == osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS
                ? "up"
                : "down")
            << std::endl;
        return;
    }

    const osg::Viewport* viewport = camera.getViewport();
    osg::notify(osg::NOTICE)
        << "[CameraMath] event=(" << event.getX() << ", " << event.getY()
        << ") yOrientation="
        << (event.getMouseYOrientation()
                == osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS
            ? "up"
            : "down")
        << " framebuffer=(" << pixelX << ", " << pixelY
        << ") viewport=(" << viewport->x() << ", " << viewport->y()
        << ", " << viewport->width() << ", " << viewport->height()
        << ") rayOrigin=(" << rayOrigin.x() << ", " << rayOrigin.y()
        << ", " << rayOrigin.z() << ") rayDirection=("
        << rayDirection.x() << ", " << rayDirection.y() << ", "
        << rayDirection.z() << ")" << std::endl;
}

void CesiumCameraManipulator::logRotationPivot(const char* source) const
{
    /*qInfo().noquote() << QStringLiteral(
        "[CameraRotatePivot] source=%1 pivot=(%2,%3,%4)")
        .arg(QString::fromLatin1(source ? source : "Unknown"))
        .arg(m_rotationPivot.x(), 0, 'g', 17)
        .arg(m_rotationPivot.y(), 0, 'g', 17)
        .arg(m_rotationPivot.z(), 0, 'g', 17);*/
}

void CesiumCameraManipulator::setPostViewUpdateCallback(
    PostViewUpdateCallback callback)
{
    m_postViewUpdateCallback = std::move(callback);
}

void CesiumCameraManipulator::setMathDebugLoggingEnabled(bool enabled)
{
    m_mathDebugLoggingEnabled = enabled;
}

void CesiumCameraManipulator::setDepthBufferPicker(DepthBufferPicker* picker)
{
    if (m_depthBufferPicker.valid()) {
        m_depthBufferPicker->clear();
    }
    m_depthBufferPicker = picker;
    invalidatePickRequests();
}

void CesiumCameraManipulator::setPickDebugVisualizer(
    PickDebugVisualizer* visualizer)
{
    m_pickDebugVisualizer = visualizer;
    if (m_pickDebugVisualizer.valid()) {
        m_pickDebugVisualizer->clear();
        m_pickDebugVisualizer->setVisible(m_pickDebugVisible);
    }
}

void CesiumCameraManipulator::setPickDebugVisible(bool visible)
{
    m_pickDebugVisible = visible;
    if (m_pickDebugVisualizer.valid()) {
        m_pickDebugVisualizer->setVisible(visible);
    }
}

bool CesiumCameraManipulator::pickDebugVisible() const
{
    return m_pickDebugVisible;
}

void CesiumCameraManipulator::setIgnoreHorizontalRotationInput(bool ignore)
{
    m_ignoreHorizontalRotationInput = ignore;
}

bool CesiumCameraManipulator::ignoreHorizontalRotationInput() const
{
    return m_ignoreHorizontalRotationInput;
}

void CesiumCameraManipulator::setIgnoreVerticalRotationInput(bool ignore)
{
    m_ignoreVerticalRotationInput = ignore;
}

bool CesiumCameraManipulator::ignoreVerticalRotationInput() const
{
    return m_ignoreVerticalRotationInput;
}

void CesiumCameraManipulator::invalidatePickRequests()
{
    ++m_pickGeneration;
    if (m_pickGeneration == 0) {
        m_pickGeneration = 1;
    }
    m_nextPickSequence = 1;
    m_latestPickSequence = 0;
    m_pendingGestureSequence = 0;
    m_pendingZoomSequence = 0;
    m_pendingZoomWheelSteps = 0.0;
    m_latestGestureEvent = nullptr;
    m_interactionMode = InteractionMode::None;
    m_pickRequestOutstanding = false;
    if (m_depthBufferPicker.valid()) {
        m_depthBufferPicker->clear();
    }
    if (m_pickDebugVisualizer.valid()) {
        m_pickDebugVisualizer->clear();
    }
}

void CesiumCameraManipulator::invalidatePendingPickSequence()
{
    if (m_nextPickSequence == std::numeric_limits<std::uint64_t>::max()) {
        ++m_pickGeneration;
        if (m_pickGeneration == 0) {
            m_pickGeneration = 1;
        }
        m_nextPickSequence = 1;
    }

    // Reserve a token without submitting a request. A result already being
    // produced by PostDraw can then no longer match the active sequence.
    m_latestPickSequence = m_nextPickSequence++;
    m_pendingGestureSequence = 0;
    m_pendingZoomSequence = 0;
    m_pendingZoomWheelSteps = 0.0;
    m_latestGestureEvent = nullptr;
    m_pickRequestOutstanding = false;
    if (m_depthBufferPicker.valid()) {
        m_depthBufferPicker->clear();
    }
}

bool CesiumCameraManipulator::requestDepthPick(
    const osgGA::GUIEventAdapter& event,
    const osg::Camera& camera,
    PickAction action,
    double wheelSteps,
    std::uint64_t& sequence)
{
    sequence = 0;
    if (!m_depthBufferPicker.valid()) {
        return false;
    }

    if (m_nextPickSequence == std::numeric_limits<std::uint64_t>::max()) {
        ++m_pickGeneration;
        if (m_pickGeneration == 0) {
            m_pickGeneration = 1;
        }
        m_nextPickSequence = 1;
    }
    sequence = m_nextPickSequence++;
    m_latestPickSequence = sequence;

    int pixelX = 0;
    int pixelY = 0;
    if (!CameraMath::eventToFramebufferPixel(
            event, camera, pixelX, pixelY)) {
        m_pickRequestOutstanding = false;
        if (m_pickDebugVisualizer.valid()) {
            m_pickDebugVisualizer->clear();
        }
        return false;
    }

    DepthPickRequest request;
    request.generation = m_pickGeneration;
    request.sequence = sequence;
    request.pixelX = pixelX;
    request.pixelY = pixelY;
    request.action = action;
    request.wheelSteps = wheelSteps;
    m_depthBufferPicker->requestPick(request);
    m_pickRequestOutstanding = true;
    return true;
}

void CesiumCameraManipulator::resolvePendingGestureFallback()
{
    if (m_interactionMode == InteractionMode::PendingPan) {
        m_panPlanePoint = m_pose.focusPoint;
        m_panPlaneNormal = m_pressForward;
        m_interactionMode = InteractionMode::Pan;
    } else if (m_interactionMode == InteractionMode::PendingRotate) {
        m_rotationPivot = m_pose.focusPoint;
        logRotationPivot("DefaultFocusPoint");
        m_interactionMode = InteractionMode::Rotate;
    }
    m_pendingGestureSequence = 0;
}

bool CesiumCameraManipulator::consumeDepthPickResult(
    const osg::Camera* camera)
{
    if (!m_depthBufferPicker.valid()) {
        return false;
    }

    DepthPickResult result;
    bool consumedCurrentResult = false;
    while (m_depthBufferPicker->consumeResult(result)) {
        if (!DepthBufferPicker::isResultCurrent(
                result, m_pickGeneration, m_latestPickSequence)) {
            continue;
        }

        m_pickRequestOutstanding = false;
        consumedCurrentResult = true;
        const bool validHit = result.hitScene && finiteVec(result.worldPoint);
        if (m_pickDebugVisualizer.valid()) {
            if (validHit) {
                m_pickDebugVisualizer->show(result.worldPoint, result.action);
            } else {
                m_pickDebugVisualizer->clear();
            }
        }

        if (result.action == PickAction::Zoom
            && result.sequence == m_pendingZoomSequence) {
            osg::Vec3d zoomPivot;
            const double hitDistance = validHit
                ? (result.worldPoint - m_pose.eye).length()
                : 0.0;
            if (validHit && std::isfinite(hitDistance)
                && hitDistance > 1.0e-12) {
                zoomPivot = result.worldPoint;
            } else {
                osg::Vec3d rayDirection = m_pendingZoomRayDirection;
                const double rayLength = rayDirection.length();
                if (!finiteVec(rayDirection) || !std::isfinite(rayLength)
                    || rayLength <= 1.0e-12) {
                    m_pendingZoomSequence = 0;
                    m_pendingZoomWheelSteps = 0.0;
                    continue;
                }
                rayDirection /= rayLength;
                zoomPivot = m_pose.eye + rayDirection * m_focusDistance;
            }

            applyZoom(zoomPivot, result.wheelSteps);
            m_pendingZoomSequence = 0;
            m_pendingZoomWheelSteps = 0.0;
            continue;
        }

        if (result.action == PickAction::BeginPan
            && result.sequence == m_pendingGestureSequence
            && m_interactionMode == InteractionMode::PendingPan) {
            m_panPlanePoint = validHit
                ? result.worldPoint
                : m_pose.focusPoint;
            m_panPlaneNormal = m_pressForward;
            m_interactionMode = InteractionMode::Pan;
            m_pendingGestureSequence = 0;
            if (camera && m_latestGestureEvent.valid()) {
                updatePan(*m_latestGestureEvent, *camera);
            }
            continue;
        }

        if (result.action == PickAction::BeginRotate
            && result.sequence == m_pendingGestureSequence
            && m_interactionMode == InteractionMode::PendingRotate) {
           /* qInfo().noquote()
                << "[DepthPick] action=BeginRotate"
                << "outcome=" << depthPickOutcomeName(result.outcome)
                << QStringLiteral("request=(%1,%2)")
                       .arg(result.requestedPixelX)
                       .arg(result.requestedPixelY)
                << QStringLiteral("viewport=(%1,%2,%3,%4)")
                       .arg(result.viewportX)
                       .arg(result.viewportY)
                       .arg(result.viewportWidth)
                       .arg(result.viewportHeight)
                << QStringLiteral("region=(%1,%2,%3,%4)")
                       .arg(result.readX)
                       .arg(result.readY)
                       .arg(result.readWidth)
                       .arg(result.readHeight)
                << "validSamples=" << result.validDepthSampleCount
                << QStringLiteral("depthRange=(%1,%2)")
                       .arg(result.minimumReadDepth, 0, 'g', 9)
                       .arg(result.maximumReadDepth, 0, 'g', 9)
                << QStringLiteral("selected=(%1,%2,%3)")
                       .arg(result.selectedPixelX)
                       .arg(result.selectedPixelY)
                       .arg(result.selectedDepth, 0, 'g', 9)
                << "fbo=" << result.framebufferBinding
                << "readFbo=" << result.readFramebufferBinding
                << "depthBits=" << result.depthBits
                << "samples=" << result.samples
                << "glError=0x"
                << QString::number(result.glError, 16);*/
            m_rotationPivot = validHit
                ? result.worldPoint
                : m_pose.focusPoint;
            logRotationPivot(validHit
                ? "DepthUnprojection"
                : "DefaultFocusPoint");
            m_interactionMode = InteractionMode::Rotate;
            m_pendingGestureSequence = 0;
            if (m_latestGestureEvent.valid()) {
                const double deltaX =
                    static_cast<double>(m_latestGestureEvent->getX())
                    - m_lastMouseX;
                const double deltaY =
                    static_cast<double>(m_latestGestureEvent->getY())
                    - m_lastMouseY;
                m_lastMouseX = m_latestGestureEvent->getX();
                m_lastMouseY = m_latestGestureEvent->getY();
                updateRotate(deltaX, deltaY);
            }
        }
    }
    return consumedCurrentResult;
}

void CesiumCameraManipulator::home(const osgGA::GUIEventAdapter&,
                                   osgGA::GUIActionAdapter& action)
{
    const osg::View* view = action.asView();
    applyHomePose(view ? view->getCamera() : nullptr);
    action.requestRedraw();
    action.requestContinuousUpdate(false);
}

void CesiumCameraManipulator::home(double)
{
    applyHomePose(nullptr);
}

bool CesiumCameraManipulator::computeHomePose(const osg::Camera* camera,
                                              CameraPose& pose,
                                              double& focusDistance) const
{
    const osg::Node* sceneNode = m_sceneNode.get();
    if (!sceneNode) {
        return false;
    }

    const osg::BoundingSphere bound = sceneNode->getBound();
    if (!bound.valid() || !finiteVec(bound.center())
        || !std::isfinite(bound.radius())) {
        return false;
    }

    const double radius = std::max(static_cast<double>(bound.radius()),
                                   MinimumHomeRadius);
    double distance = std::max(1.0, radius * DefaultHomeDistanceScale);

    if (camera) {
        double fovYDegrees = 0.0;
        double aspectRatio = 0.0;
        double nearPlane = 0.0;
        double farPlane = 0.0;
        if (camera->getProjectionMatrixAsPerspective(
                fovYDegrees, aspectRatio, nearPlane, farPlane)
            && std::isfinite(fovYDegrees) && fovYDegrees > 0.0
            && fovYDegrees < 180.0
            && std::isfinite(aspectRatio) && aspectRatio > 0.0) {
            const double verticalHalfAngle = osg::DegreesToRadians(fovYDegrees * 0.5);
            const double horizontalHalfAngle = std::atan(
                std::tan(verticalHalfAngle) * aspectRatio);
            const double limitingHalfAngle = std::min(verticalHalfAngle,
                                                      horizontalHalfAngle);
            const double sine = std::sin(limitingHalfAngle);
            if (std::isfinite(sine) && sine > 1.0e-6) {
                distance = std::max(1.0, radius / sine * HomeFramingPadding);
            }
        }
    }

    distance = std::clamp(
        distance, m_minFocusDistance, m_maxFocusDistance);

    const osg::Vec3d center = bound.center();
    const osg::Vec3d eye = center + osg::Vec3d(0.0, -distance, 0.0);
    osg::Quat rotation;
    if (!rotationFromLookAt(
            eye, center, osg::Vec3d(0.0, 0.0, 1.0), rotation)) {
        return false;
    }

    pose.eye = eye;
    pose.rotation = rotation;
    pose.focusPoint = center;
    focusDistance = distance;
    return true;
}

void CesiumCameraManipulator::applyHomePose(const osg::Camera* camera)
{
    m_interactionMode = InteractionMode::None;
    invalidatePickRequests();
    CameraPose homePose;
    double homeFocusDistance = 0.0;
    if (computeHomePose(camera, homePose, homeFocusDistance)) {
        m_pose = homePose;
        m_focusDistance = homeFocusDistance;
    }
}
