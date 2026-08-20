#pragma once

#include <osg/Group>
#include <osg/ref_ptr>
#include <osgQOpenGL/osgQOpenGLWidget>

#include <QString>

#include <memory>

class SceneManager;
class PointCloudDataset;
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
    void clearPointCloud();

    void setPointSize(float pointSize);
    float pointSize() const;

signals:
    void pointCloudChanged(const QString& filePath, quint64 pointCount);
    void viewerInitialized();

private:
    void initializeViewer();

    std::unique_ptr<SceneManager> m_sceneManager;
    osg::ref_ptr<osg::Group> m_root;
    bool m_initialized = false;
    float m_pointSize = 3.0f;
};
