#include "viewer/OsgViewWidget.h"

#include "pointcloud/PointCloudRuntime.h"
#include "pointcloud/PointCloudUpdateCallback.h"
#include "viewer/PotreeRenderMasks.h"
#include "viewer/SceneManager.h"
#include "viewer/camera/CameraMath.h"
#include "viewer/camera/CesiumCameraManipulator.h"
#include "viewer/camera/DepthBufferPicker.h"
#include "viewer/camera/PickDebugVisualizer.h"

#include <OpenThreads/ReadWriteMutex>

#include <osg/Camera>
#include <osg/CullSettings>
#include <osg/Matrixd>
#include <osg/Viewport>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <QFile>
#include <QDebug>
#include <QMetaObject>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QSurfaceFormat>

#include <cmath>
#include <functional>
#include <utility>

namespace
{
constexpr double kDiagnosticMaxFrameRate = 60.0;
constexpr bool kDiagnosticFrameRateLimitEnabled = false;
constexpr bool kPaintGlDiagnosticsEnabled = false;

class ProjectionTrackballManipulator final
    : public osgGA::TrackballManipulator
{
public:
    using ProjectionCallback = std::function<void(osg::Camera&)>;

    explicit ProjectionTrackballManipulator(ProjectionCallback callback)
        : m_callback(std::move(callback))
    {
    }

    void updateCamera(osg::Camera& camera) override
    {
        osgGA::TrackballManipulator::updateCamera(camera);
        if (m_callback) {
            m_callback(camera);
        }
    }

private:
    ProjectionCallback m_callback;
};

QString profileName(QSurfaceFormat::OpenGLContextProfile profile)
{
    switch (profile) {
    case QSurfaceFormat::CoreProfile:
        return QStringLiteral("Core");
    case QSurfaceFormat::CompatibilityProfile:
        return QStringLiteral("Compatibility");
    case QSurfaceFormat::NoProfile:
        return QStringLiteral("NoProfile");
    }
    return QStringLiteral("Unknown");
}

QString formatSummary(const QSurfaceFormat& format)
{
    return QStringLiteral(
               "OpenGL %1.%2 %3, depth=%4, stencil=%5, samples=%6, swapInterval=%7")
        .arg(format.majorVersion())
        .arg(format.minorVersion())
        .arg(profileName(format.profile()))
        .arg(format.depthBufferSize())
        .arg(format.stencilBufferSize())
        .arg(format.samples())
        .arg(format.swapInterval());
}

QString glString(QOpenGLFunctions* functions, GLenum name)
{
    if (!functions) {
        return QStringLiteral("unavailable");
    }
    const GLubyte* value = functions->glGetString(name);
    return value ? QString::fromLatin1(reinterpret_cast<const char*>(value))
                 : QStringLiteral("unavailable");
}
} // namespace

OsgViewWidget::OsgViewWidget(QWidget* parent)
    : osgQOpenGLWidget(parent)
    , m_sceneManager(std::make_unique<SceneManager>())
    , m_runtime(std::make_unique<PointCloudRuntime>(m_sceneManager.get()))
    , m_root(m_sceneManager->root())
{
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    m_runtime->setCompletionCallback([this]() {
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
    });
    connect(this, &osgQOpenGLWidget::initialized,
            this, [this]() { initializeViewer(); });
}

OsgViewWidget::~OsgViewWidget()
{
    osgViewer::Viewer* viewer = getOsgViewer();
    osg::Camera* camera = viewer ? viewer->getCamera() : nullptr;
    if (camera && m_depthBufferPicker.valid()) {
        camera->removePostDrawCallback(m_depthBufferPicker.get());
    }
    if (m_cameraManipulator.valid()) {
        m_cameraManipulator->setPostViewUpdateCallback({});
        m_cameraManipulator->setDepthBufferPicker(nullptr);
        m_cameraManipulator->setPickDebugVisualizer(nullptr);
    }
    if (viewer) {
        viewer->setCameraManipulator(nullptr, false);
    }
    if (viewer && m_pickDebugCamera.valid()) {
        const unsigned int slaveIndex = viewer->findSlaveIndexForCamera(
            m_pickDebugCamera.get());
        if (slaveIndex < viewer->getNumSlaves()) {
            viewer->removeSlave(slaveIndex);
        }
        m_pickDebugCamera->setGraphicsContext(nullptr);
    }
}

void OsgViewWidget::paintGL()
{
    if constexpr (!kPaintGlDiagnosticsEnabled) {
        osgQOpenGLWidget::paintGL();
        return;
    }

    if (!m_paintDiagnosticTimer.isValid()) {
        m_paintDiagnosticTimer.start();
    }

    const qint64 startNs = m_paintDiagnosticTimer.nsecsElapsed();
    const double intervalMs = m_lastPaintStartNs > 0
        ? static_cast<double>(startNs - m_lastPaintStartNs) / 1000000.0
        : 0.0;
    const double outsideMs = m_lastPaintEndNs > 0
        ? static_cast<double>(startNs - m_lastPaintEndNs) / 1000000.0
        : 0.0;

    osgQOpenGLWidget::paintGL();

    const qint64 finishNs = m_paintDiagnosticTimer.nsecsElapsed();
    const double paintMs = static_cast<double>(finishNs - startNs) / 1000000.0;
    const bool longFrame = intervalMs >= 25.0 || paintMs >= 25.0 || outsideMs >= 25.0;
    const bool mayLog = m_lastPaintDiagnosticNs == 0
        || static_cast<double>(finishNs - m_lastPaintDiagnosticNs) / 1000000.0 >= 500.0;
    if (longFrame && mayLog) {
        m_lastPaintDiagnosticNs = finishNs;
        qWarning().nospace()
            << "paintGL diagnostic intervalMs=" << intervalMs
            << " paintMs=" << paintMs
            << " outsideMs=" << outsideMs;
    }

    m_lastPaintStartNs = startNs;
    m_lastPaintEndNs = m_paintDiagnosticTimer.nsecsElapsed();
}

bool OsgViewWidget::loadPointCloud(const QString& filePath, QString* errorMessage)
{
    if (!m_initialized) {
        if (errorMessage) {
            *errorMessage = tr("The OSG viewer is not initialized yet.");
        }
        return false;
    }

    const QByteArray nativePath = QFile::encodeName(filePath);
    QString error;

    {
        OpenThreads::ScopedWriteLock lock(*mutex());
        if (!m_sceneManager->loadPointCloud(nativePath.constData(), filePath, m_pointSize, &error)) {
            if (errorMessage) {
                *errorMessage = error;
            }
            return false;
        }
    }

    getOsgViewer()->home();
    update();
    emit pointCloudChanged(m_sceneManager->currentFilePath(), m_sceneManager->pointCount());
    return true;
}

bool OsgViewWidget::loadPotreeNode(const PointCloudDataset& dataset,
                                   const PointCloudNodeData& nodeData,
                                   QString* errorMessage)
{
    if (!m_initialized) {
        if (errorMessage) {
            *errorMessage = tr("The OSG viewer is not initialized yet.");
        }
        return false;
    }

    QString error;
    {
        OpenThreads::ScopedWriteLock lock(*mutex());
        if (!m_sceneManager->loadPotreeNode(dataset, nodeData, m_pointSize, &error)) {
            if (errorMessage) {
                *errorMessage = error;
            }
            return false;
        }
    }

    getOsgViewer()->home();
    update();
    emit pointCloudChanged(m_sceneManager->currentFilePath(), m_sceneManager->pointCount());
    return true;
}

bool OsgViewWidget::loadPotreeDataset(std::shared_ptr<PointCloudDataset> dataset,
                                      QString* errorMessage)
{
    if (!m_initialized) {
        if (errorMessage) {
            *errorMessage = tr("The OSG viewer is not initialized yet.");
        }
        return false;
    }

    if (!dataset || !dataset->root) {
        if (errorMessage) {
            *errorMessage = tr("The Potree dataset has no octree root.");
        }
        return false;
    }

    {
        OpenThreads::ScopedWriteLock lock(*mutex());
        m_runtime->openDataset(dataset, m_pointSize);
    }

    getOsgViewer()->home();
    update();
    emit pointCloudChanged(dataset->sourcePath, dataset->totalPoints);
    return true;
}

void OsgViewWidget::clearPointCloud()
{
    if (m_cameraManipulator.valid()) {
        m_cameraManipulator->invalidatePickRequests();
    }
    {
        OpenThreads::ScopedWriteLock lock(*mutex());
        m_runtime->clear();
        m_sceneManager->clear();
    }

    update();
    emit pointCloudChanged(QString(), 0);
}

void OsgViewWidget::setPointSize(float pointSize)
{
    m_pointSize = pointSize;

    {
        OpenThreads::ScopedWriteLock lock(*mutex());
        m_sceneManager->setPointSize(pointSize);
    }

    update();
}

float OsgViewWidget::pointSize() const
{
    return m_pointSize;
}

bool OsgViewWidget::advancedRenderingAvailable() const
{
    return m_advancedRenderingAvailable;
}

QString OsgViewWidget::advancedRenderingStatus() const
{
    return m_advancedRenderingStatus;
}

void OsgViewWidget::setPotreeColorMode(PotreeColorMode mode)
{
    {
        OpenThreads::ScopedWriteLock lock(*mutex());
        m_sceneManager->setPotreeColorMode(mode);
    }

    update();
}

void OsgViewWidget::setPotreeBoundingBoxesVisible(bool visible)
{
    m_showPotreeBoundingBoxes = visible;
    if (m_initialized) {
        applyPotreeRenderMask();
        update();
    }
}

void OsgViewWidget::setPickDebugVisible(bool visible)
{
    m_pickDebugVisible = visible;
    if (m_cameraManipulator.valid()) {
        m_cameraManipulator->setPickDebugVisible(visible);
    }
    update();
}

bool OsgViewWidget::pickDebugVisible() const
{
    return m_pickDebugVisible;
}

void OsgViewWidget::setIgnoreHorizontalRotationInput(bool ignore)
{
    m_ignoreHorizontalRotationInput = ignore;
    if (m_cameraManipulator.valid()) {
        m_cameraManipulator->setIgnoreHorizontalRotationInput(ignore);
    }
    update();
}

void OsgViewWidget::setIgnoreVerticalRotationInput(bool ignore)
{
    m_ignoreVerticalRotationInput = ignore;
    if (m_cameraManipulator.valid()) {
        m_cameraManipulator->setIgnoreVerticalRotationInput(ignore);
    }
    update();
}

void OsgViewWidget::setUseCesiumCamera(bool useCesium)
{
    if (m_useCesiumCamera == useCesium) {
        return;
    }

    m_useCesiumCamera = useCesium;
    applyCameraController();
}

bool OsgViewWidget::usingCesiumCamera() const
{
    return m_useCesiumCamera;
}

void OsgViewWidget::applyCameraController()
{
    if (!m_initialized || !m_cameraManipulator.valid()
        || !m_trackballManipulator.valid()) {
        return;
    }

    osgViewer::Viewer* viewer = getOsgViewer();
    osgGA::CameraManipulator* current = viewer->getCameraManipulator();
    const osg::Matrixd cameraMatrix = current
        ? current->getMatrix()
        : osg::Matrixd::inverse(viewer->getCamera()->getViewMatrix());

    m_cameraManipulator->invalidatePickRequests();
    if (m_pickDebugVisualizer.valid()) {
        m_pickDebugVisualizer->clear();
    }

    if (m_useCesiumCamera) {
        m_cameraManipulator->setByMatrix(cameraMatrix);
        m_cameraManipulator->setFocusDistance(
            m_trackballManipulator->getDistance());
        viewer->setCameraManipulator(m_cameraManipulator.get(), false);
    } else {
        const osg::Vec3d eye = cameraMatrix.getTrans();
        const osg::Quat rotation = cameraMatrix.getRotate();
        const osg::Vec3d center = eye
            + rotation * osg::Vec3d(
                0.0, 0.0, -m_cameraManipulator->focusDistance());
        const osg::Vec3d up = rotation * osg::Vec3d(0.0, 1.0, 0.0);
        m_trackballManipulator->setTransformation(eye, center, up);
        viewer->setCameraManipulator(m_trackballManipulator.get(), false);
    }

    update();
}

void OsgViewWidget::applyPotreeRenderMask()
{
    osg::Camera* camera = getOsgViewer()->getCamera();
    osg::Node::NodeMask cullMask = camera->getCullMask();
    cullMask |= PotreeRenderMasks::Points;
    if (m_showPotreeBoundingBoxes) {
        cullMask |= PotreeRenderMasks::BoundingBoxes;
    } else {
        cullMask &= ~PotreeRenderMasks::BoundingBoxes;
    }
    camera->setCullMask(cullMask);
}

void OsgViewWidget::initializeViewer()
{
    osgViewer::Viewer* viewer = getOsgViewer();
    viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    if constexpr (kDiagnosticFrameRateLimitEnabled) {
        viewer->setRunMaxFrameRate(kDiagnosticMaxFrameRate);
    }
    viewer->setSceneData(m_root.get());

    osg::Camera* camera = viewer->getCamera();
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    if (const osg::Viewport* viewport = camera->getViewport()) {
        if (viewport->width() > 0.0 && viewport->height() > 0.0) {
            m_aspectRatio = viewport->width() / viewport->height();
        }
    }
    camera->setProjectionMatrixAsPerspective(
        m_fovYDegrees, m_aspectRatio, m_nearPlane, m_farPlane);

    m_depthBufferPicker = new DepthBufferPicker;
    camera->addPostDrawCallback(m_depthBufferPicker.get());

    m_pickDebugVisualizer = new PickDebugVisualizer;
    m_pickDebugVisualizer->setVisible(m_pickDebugVisible);
    m_pickDebugCamera = new osg::Camera;
    m_pickDebugCamera->setName("CameraPickDebugPostRenderCamera");
    m_pickDebugCamera->setReferenceFrame(osg::Transform::RELATIVE_RF);
    m_pickDebugCamera->setRenderOrder(osg::Camera::POST_RENDER, 1000);
    m_pickDebugCamera->setClearMask(0);
    m_pickDebugCamera->setAllowEventFocus(false);
    m_pickDebugCamera->setComputeNearFarMode(
        osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    m_pickDebugCamera->setCullMask(PotreeRenderMasks::PickDebug);
    m_pickDebugCamera->setGraphicsContext(camera->getGraphicsContext());
    if (const osg::Viewport* viewport = camera->getViewport()) {
        m_pickDebugCamera->setViewport(new osg::Viewport(*viewport));
    }
    m_pickDebugCamera->addChild(m_pickDebugVisualizer->node());
    viewer->addSlave(m_pickDebugCamera.get(), false);

    m_cameraManipulator = new CesiumCameraManipulator;
    m_cameraManipulator->setMathDebugLoggingEnabled(false);
    m_cameraManipulator->setPostViewUpdateCallback(
        [this](osg::Camera& updatedCamera) {
            updatePerspectiveProjection(updatedCamera);
        });
    m_cameraManipulator->setDepthBufferPicker(m_depthBufferPicker.get());
    m_cameraManipulator->setPickDebugVisualizer(m_pickDebugVisualizer.get());
    m_cameraManipulator->setPickDebugVisible(m_pickDebugVisible);
    m_cameraManipulator->setIgnoreHorizontalRotationInput(
        m_ignoreHorizontalRotationInput);
    m_cameraManipulator->setIgnoreVerticalRotationInput(
        m_ignoreVerticalRotationInput);

    m_trackballManipulator = new ProjectionTrackballManipulator(
        [this](osg::Camera& updatedCamera) {
            updatePerspectiveProjection(updatedCamera);
        });

    viewer->setCameraManipulator(
        m_useCesiumCamera
            ? static_cast<osgGA::CameraManipulator*>(m_cameraManipulator.get())
            : static_cast<osgGA::CameraManipulator*>(m_trackballManipulator.get()));
    viewer->addEventHandler(new osgViewer::StatsHandler);
    camera->setClearColor(osg::Vec4(0.06f, 0.07f, 0.08f, 1.0f));
    applyPotreeRenderMask();
    m_fpsTimer.start();
    m_root->setUpdateCallback(new PointCloudUpdateCallback(
        m_runtime.get(),
        viewer,
        &m_pointSize,
        [this]() { recordRenderedFrame(); }));

    initializeRenderingCapabilities();

    m_initialized = true;
    emit viewerInitialized();
    emit renderingCapabilitiesChanged(
        m_advancedRenderingAvailable, m_advancedRenderingStatus);
}

void OsgViewWidget::initializeRenderingCapabilities()
{
    const QSurfaceFormat requestedFormat = QSurfaceFormat::defaultFormat();
    QOpenGLContext* context = QOpenGLContext::currentContext();
    if (!context) {
        m_advancedRenderingStatus = tr("No current OpenGL context; using fixed-function Potree rendering.");
        qWarning().noquote() << m_advancedRenderingStatus;
        return;
    }

    const QSurfaceFormat actualFormat = context->format();
    QOpenGLFunctions* functions = context->functions();
    qInfo().noquote() << "Requested context:" << formatSummary(requestedFormat);
    qInfo().noquote() << "Actual context:" << formatSummary(actualFormat);
    qInfo().noquote() << "GL_VENDOR:" << glString(functions, GL_VENDOR);
    qInfo().noquote() << "GL_RENDERER:" << glString(functions, GL_RENDERER);
    qInfo().noquote() << "GL_VERSION:" << glString(functions, GL_VERSION);
    qInfo().noquote() << "GL_SHADING_LANGUAGE_VERSION:"
                      << glString(functions, GL_SHADING_LANGUAGE_VERSION);

    const bool versionSupported = actualFormat.majorVersion() > 3
        || (actualFormat.majorVersion() == 3 && actualFormat.minorVersion() >= 3);
    const bool compatibilityProfile = actualFormat.profile()
        == QSurfaceFormat::CompatibilityProfile;
    if (context->isOpenGLES() || !versionSupported || !compatibilityProfile) {
        m_advancedRenderingStatus = tr(
            "OpenGL 3.3 Compatibility is unavailable; using fixed-function Potree rendering.");
        qWarning().noquote() << m_advancedRenderingStatus;
        return;
    }

    QString shaderError;
    if (!m_sceneManager->initializePotreeShader(&shaderError)) {
        m_advancedRenderingStatus = tr("Point cloud Shader initialization failed: %1")
                                        .arg(shaderError);
        qWarning().noquote() << m_advancedRenderingStatus;
        return;
    }

    m_advancedRenderingAvailable = true;
    m_advancedRenderingStatus = tr("OpenGL 3.3 point cloud Shader enabled.");
    qInfo().noquote() << m_advancedRenderingStatus;
}

void OsgViewWidget::recordRenderedFrame()
{
    if (!m_fpsTimer.isValid()) {
        m_fpsTimer.start();
    }

    ++m_fpsFrameCount;
    const qint64 elapsedMs = m_fpsTimer.elapsed();
    if (elapsedMs < 1000) {
        return;
    }

    const double fps = static_cast<double>(m_fpsFrameCount) * 1000.0
        / static_cast<double>(elapsedMs);
    m_fpsFrameCount = 0;
    m_fpsTimer.restart();
    emit fpsChanged(fps);
}

void OsgViewWidget::updatePerspectiveProjection(osg::Camera& camera)
{
    const osg::Viewport* viewport = camera.getViewport();
    if (viewport && std::isfinite(viewport->width())
        && std::isfinite(viewport->height())
        && viewport->width() > 0.0 && viewport->height() > 0.0) {
        m_aspectRatio = viewport->width() / viewport->height();
    }

    osg::Matrixd cameraMatrix;
    if (cameraMatrix.invert(camera.getViewMatrix())) {
        double nearPlane = 0.0;
        double farPlane = 0.0;
        const double projectionFocusDistance =
            !m_useCesiumCamera && m_trackballManipulator.valid()
                ? m_trackballManipulator->getDistance()
                : (m_cameraManipulator.valid()
                    ? m_cameraManipulator->focusDistance()
                    : 1.0);
        if (CameraMath::computePerspectiveNearFar(
                cameraMatrix.getTrans(),
                m_root->getBound(),
                projectionFocusDistance,
                nearPlane,
                farPlane)) {
            m_nearPlane = nearPlane;
            m_farPlane = farPlane;
        }
    }

    camera.setProjectionMatrixAsPerspective(
        m_fovYDegrees, m_aspectRatio, m_nearPlane, m_farPlane);

    if (m_pickDebugCamera.valid() && viewport) {
        const osg::Viewport* debugViewport = m_pickDebugCamera->getViewport();
        if (!debugViewport
            || debugViewport->x() != viewport->x()
            || debugViewport->y() != viewport->y()
            || debugViewport->width() != viewport->width()
            || debugViewport->height() != viewport->height()) {
            m_pickDebugCamera->setViewport(new osg::Viewport(*viewport));
        }
    }
}
