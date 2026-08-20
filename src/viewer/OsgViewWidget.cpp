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
#include <QMetaObject>

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
    m_root->setUpdateCallback(new PointCloudUpdateCallback(m_runtime.get(), viewer, &m_pointSize));

    m_initialized = true;
    emit viewerInitialized();
}
