#pragma once

#include <QMainWindow>

class QAction;
class QLabel;
class QDoubleSpinBox;
class OsgViewWidget;

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
    void closePointCloud();
    void updatePointCloudStatus(const QString& filePath, quint64 pointCount);

    OsgViewWidget* m_viewWidget = nullptr;
    QAction* m_openAction = nullptr;
    QAction* m_closeAction = nullptr;
    QAction* m_exitAction = nullptr;
    QLabel* m_fileLabel = nullptr;
    QLabel* m_pointCountLabel = nullptr;
    QDoubleSpinBox* m_pointSizeSpinBox = nullptr;
};
