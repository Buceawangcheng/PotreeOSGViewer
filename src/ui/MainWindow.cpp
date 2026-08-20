#include "ui/MainWindow.h"

#include "viewer/OsgViewWidget.h"

#include <QAction>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_viewWidget(new OsgViewWidget(this))
{
    setWindowTitle("PotreeOSGViewer");
    setCentralWidget(m_viewWidget);

    createActions();
    createMenus();
    createToolbar();
    createStatusBar();

    connect(m_viewWidget, &OsgViewWidget::pointCloudChanged,
            this, [this](const QString& filePath, quint64 pointCount) {
                updatePointCloudStatus(filePath, pointCount);
            });
}

void MainWindow::createActions()
{
    m_openAction = new QAction(tr("&Open Point Cloud..."), this);
    m_openAction->setShortcut(QKeySequence::Open);
    connect(m_openAction, &QAction::triggered, this, [this]() { openPointCloud(); });

    m_closeAction = new QAction(tr("&Close Point Cloud"), this);
    m_closeAction->setEnabled(false);
    connect(m_closeAction, &QAction::triggered, this, [this]() { closePointCloud(); });

    m_exitAction = new QAction(tr("E&xit"), this);
    m_exitAction->setShortcut(QKeySequence::Quit);
    connect(m_exitAction, &QAction::triggered, this, &QWidget::close);
}

void MainWindow::createMenus()
{
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(m_openAction);
    fileMenu->addAction(m_closeAction);
    fileMenu->addSeparator();
    fileMenu->addAction(m_exitAction);
}

void MainWindow::createToolbar()
{
    QToolBar* toolbar = addToolBar(tr("Point Cloud"));
    toolbar->setObjectName("PointCloudToolbar");
    toolbar->addAction(m_openAction);
    toolbar->addAction(m_closeAction);
    toolbar->addSeparator();
    toolbar->addWidget(new QLabel(tr("Point Size"), toolbar));

    m_pointSizeSpinBox = new QDoubleSpinBox(toolbar);
    m_pointSizeSpinBox->setRange(1.0, 20.0);
    m_pointSizeSpinBox->setSingleStep(1.0);
    m_pointSizeSpinBox->setDecimals(1);
    m_pointSizeSpinBox->setValue(m_viewWidget->pointSize());
    toolbar->addWidget(m_pointSizeSpinBox);

    connect(m_pointSizeSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double value) {
                m_viewWidget->setPointSize(static_cast<float>(value));
            });
}

void MainWindow::createStatusBar()
{
    m_fileLabel = new QLabel(tr("No point cloud"), this);
    m_pointCountLabel = new QLabel(tr("Points: 0"), this);

    statusBar()->addWidget(m_fileLabel, 1);
    statusBar()->addPermanentWidget(m_pointCountLabel);
}

void MainWindow::openPointCloud()
{
    const QString filePath = QFileDialog::getOpenFileName(
        this,
        tr("Open Point Cloud"),
        QString(),
        tr("Point cloud (*.ply);;All files (*.*)"));

    if (filePath.isEmpty()) {
        return;
    }

    QString error;
    if (!m_viewWidget->loadPointCloud(filePath, &error)) {
        QMessageBox::critical(this, tr("Open Point Cloud Failed"), error);
        return;
    }
}

void MainWindow::closePointCloud()
{
    m_viewWidget->clearPointCloud();
}

void MainWindow::updatePointCloudStatus(const QString& filePath, quint64 pointCount)
{
    if (filePath.isEmpty()) {
        m_fileLabel->setText(tr("No point cloud"));
        m_pointCountLabel->setText(tr("Points: 0"));
        m_closeAction->setEnabled(false);
        return;
    }

    m_fileLabel->setText(QFileInfo(filePath).fileName());
    m_pointCountLabel->setText(tr("Points: %1").arg(pointCount));
    m_closeAction->setEnabled(true);
}
