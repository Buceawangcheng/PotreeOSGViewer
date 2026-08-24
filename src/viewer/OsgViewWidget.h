#pragma once

#include "viewer/PotreeColorMode.h"

#include <osg/Group>
#include <osg/ref_ptr>
#include <osgQOpenGL/osgQOpenGLWidget>

#include <QElapsedTimer>
#include <QString>

#include <memory>

namespace osg {
class Camera;
}

namespace osgGA {
class TrackballManipulator;
}

class SceneManager;
class CesiumCameraManipulator;
class DepthBufferPicker;
class PickDebugVisualizer;
class PointCloudDataset;
class PointCloudRuntime;
struct PointCloudNodeData;

class OsgViewWidget : public osgQOpenGLWidget
{
    Q_OBJECT

public:
    explicit OsgViewWidget(QWidget* parent = nullptr);
    ~OsgViewWidget() override;

    bool loadPointCloud(const QString& filePath, QString* errorMessage = nullptr);
    bool loadPotreeNode(const PointCloudDataset& dataset,
                        const PointCloudNodeData& nodeData,
                        QString* errorMessage = nullptr);
    bool loadPotreeDataset(std::shared_ptr<PointCloudDataset> dataset,
                           QString* errorMessage = nullptr);
    void clearPointCloud();

    void setPointSize(float pointSize);
    float pointSize() const;
    bool advancedRenderingAvailable() const;
    QString advancedRenderingStatus() const;
    void setPotreeColorMode(PotreeColorMode mode);
    void setPotreeBoundingBoxesVisible(bool visible);
    void setPickDebugVisible(bool visible);
    bool pickDebugVisible() const;
    void setIgnoreHorizontalRotationInput(bool ignore);
    void setIgnoreVerticalRotationInput(bool ignore);
    void setUseCesiumCamera(bool useCesium);
    bool usingCesiumCamera() const;

signals:
    void pointCloudChanged(const QString& filePath, quint64 pointCount);
    void fpsChanged(double fps);
    void viewerInitialized();
    void renderingCapabilitiesChanged(bool available, const QString& status);

protected:
    void paintGL() override;

private:
    void initializeViewer();
    void initializeRenderingCapabilities();
    void applyPotreeRenderMask();
    void recordRenderedFrame();
    void updatePerspectiveProjection(osg::Camera& camera);
    void applyCameraController();

    std::unique_ptr<SceneManager> m_sceneManager;
    std::unique_ptr<PointCloudRuntime> m_runtime;
    osg::ref_ptr<osg::Group> m_root;
    osg::ref_ptr<CesiumCameraManipulator> m_cameraManipulator;
    osg::ref_ptr<osgGA::TrackballManipulator> m_trackballManipulator;
    osg::ref_ptr<DepthBufferPicker> m_depthBufferPicker;
    osg::ref_ptr<PickDebugVisualizer> m_pickDebugVisualizer;
    osg::ref_ptr<osg::Camera> m_pickDebugCamera;
    bool m_initialized = false;
    bool m_showPotreeBoundingBoxes = false;
    bool m_pickDebugVisible = false;
    bool m_ignoreHorizontalRotationInput = false;
    bool m_ignoreVerticalRotationInput = false;
    bool m_useCesiumCamera = true;
    bool m_advancedRenderingAvailable = false;
    QString m_advancedRenderingStatus;
    QElapsedTimer m_fpsTimer;
    QElapsedTimer m_paintDiagnosticTimer;
    qint64 m_lastPaintStartNs = 0;
    qint64 m_lastPaintEndNs = 0;
    qint64 m_lastPaintDiagnosticNs = 0;
    int m_fpsFrameCount = 0;
    float m_pointSize = 3.0f;
    double m_fovYDegrees = 45.0;
    double m_aspectRatio = 1.0;
    double m_nearPlane = 0.1;
    double m_farPlane = 10000.0;
};
