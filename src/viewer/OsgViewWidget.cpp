#include "viewer/OsgViewWidget.h"

#include "viewer/SceneManager.h"

#include <OpenThreads/ReadWriteMutex>

#include <osg/Camera>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>

#include <QFile>

OsgViewWidget::OsgViewWidget(QWidget* parent)
    : osgQOpenGLWidget(parent)
    , m_sceneManager(std::make_unique<SceneManager>())
    , m_root(m_sceneManager->root())
{
    setFocusPolicy(Qt::StrongFocus);
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

void OsgViewWidget::clearPointCloud()
{
    {
        OpenThreads::ScopedWriteLock lock(*mutex());
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

void OsgViewWidget::initializeViewer()
{
    osgViewer::Viewer* viewer = getOsgViewer();
    viewer->setSceneData(m_root.get());
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);
    viewer->addEventHandler(new osgViewer::StatsHandler);
    viewer->getCamera()->setClearColor(osg::Vec4(0.06f, 0.07f, 0.08f, 1.0f));

    m_initialized = true;
    emit viewerInitialized();
}
