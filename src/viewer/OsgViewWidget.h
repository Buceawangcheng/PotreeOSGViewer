#pragma once

#include "viewer/PotreeColorMode.h"

#include <osg/Group>
#include <osg/ref_ptr>
#include <osgQOpenGL/osgQOpenGLWidget>

#include <QElapsedTimer>
#include <QString>

#include <memory>

class SceneManager;
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

signals:
    void pointCloudChanged(const QString& filePath, quint64 pointCount);
    void fpsChanged(double fps);
    void viewerInitialized();
    void renderingCapabilitiesChanged(bool available, const QString& status);

private:
    void initializeViewer();
    void initializeRenderingCapabilities();
    void applyPotreeRenderMask();
    void recordRenderedFrame();

    std::unique_ptr<SceneManager> m_sceneManager;
    std::unique_ptr<PointCloudRuntime> m_runtime;
    osg::ref_ptr<osg::Group> m_root;
    bool m_initialized = false;
    bool m_showPotreeBoundingBoxes = false;
    bool m_advancedRenderingAvailable = false;
    QString m_advancedRenderingStatus;
    QElapsedTimer m_fpsTimer;
    int m_fpsFrameCount = 0;
    float m_pointSize = 3.0f;
};
