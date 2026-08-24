#pragma once

#include <QMainWindow>

#include <memory>

class QAction;
class QCheckBox;
class QLabel;
class QComboBox;
class QDoubleSpinBox;
class OsgViewWidget;
class PointCloudDataset;

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    void createActions();
    void createMenus();
    void createToolbar();
    void createStatusBar();
    void openPointCloud();
    void openPotreeMetadata();
    QString selectPotreeMetadataPath();
    void closePointCloud();
    void updatePointCloudStatus(const QString& filePath, quint64 pointCount);
    QString potreeSummaryText(const PointCloudDataset& dataset) const;

    OsgViewWidget* m_viewWidget = nullptr;
    QAction* m_openAction = nullptr;
    QAction* m_openPotreeMetadataAction = nullptr;
    QAction* m_closeAction = nullptr;
    QAction* m_exitAction = nullptr;
    QAction* m_pickDebugAction = nullptr;
    QAction* m_defaultCameraAction = nullptr;
    QAction* m_cesiumCameraAction = nullptr;
    QAction* m_ignoreHorizontalRotationAction = nullptr;
    QAction* m_ignoreVerticalRotationAction = nullptr;
    QLabel* m_fileLabel = nullptr;
    QLabel* m_pointCountLabel = nullptr;
    QLabel* m_fpsLabel = nullptr;
    QDoubleSpinBox* m_pointSizeSpinBox = nullptr;
    QComboBox* m_colorModeComboBox = nullptr;
    QCheckBox* m_boundsCheckBox = nullptr;
    std::shared_ptr<PointCloudDataset> m_potreeMetadataDataset;
};
