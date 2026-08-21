#include "viewer/OsgViewWidget.h"

#include "pointcloud/PointCloudRuntime.h"
#include "pointcloud/PointCloudUpdateCallback.h"
#include "viewer/PotreeRenderMasks.h"
#include "viewer/SceneManager.h"

#include <OpenThreads/ReadWriteMutex>

#include <osg/Camera>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>

#include <QFile>
#include <QDebug>
#include <QMetaObject>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QSurfaceFormat>

namespace
{
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
    return QStringLiteral("OpenGL %1.%2 %3, depth=%4, stencil=%5, samples=%6")
        .arg(format.majorVersion())
        .arg(format.minorVersion())
        .arg(profileName(format.profile()))
        .arg(format.depthBufferSize())
        .arg(format.stencilBufferSize())
        .arg(format.samples());
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
    m_runtime->setCompletionCallback([this]() {
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
    });
    connect(this, &osgQOpenGLWidget::initialized,
            this, [this]() { initializeViewer(); });
}

OsgViewWidget::~OsgViewWidget() = default;

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
    viewer->setSceneData(m_root.get());
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    viewer->addEventHandler(new osgViewer::StatsHandler);
    viewer->getCamera()->setClearColor(osg::Vec4(0.06f, 0.07f, 0.08f, 1.0f));
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
