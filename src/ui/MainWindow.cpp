#include "ui/MainWindow.h"

#include "pointcloud/Potree2Provider.h"
#include "viewer/OsgViewWidget.h"

#include <QAction>
#include <QDebug>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QLocale>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QStatusBar>
#include <QToolBar>

#include <utility>

namespace
{
QString formatVec3(const osg::Vec3d& value)
{
    return QStringLiteral("%1, %2, %3")
        .arg(value.x(), 0, 'f', 6)
        .arg(value.y(), 0, 'f', 6)
        .arg(value.z(), 0, 'f', 6);
}
} // namespace

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

    m_openPotreeMetadataAction = new QAction(tr("Open &Potree Dataset..."), this);
    connect(m_openPotreeMetadataAction, &QAction::triggered, this, [this]() { openPotreeMetadata(); });

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
    fileMenu->addAction(m_openPotreeMetadataAction);
    fileMenu->addAction(m_closeAction);
    fileMenu->addSeparator();
    fileMenu->addAction(m_exitAction);
}

void MainWindow::createToolbar()
{
    QToolBar* toolbar = addToolBar(tr("Point Cloud"));
    toolbar->setObjectName("PointCloudToolbar");
    toolbar->addAction(m_openAction);
    toolbar->addAction(m_openPotreeMetadataAction);
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

    m_potreeMetadataDataset.reset();
}

void MainWindow::openPotreeMetadata()
{
    const QString path = selectPotreeMetadataPath();
    if (path.isEmpty()) {
        return;
    }

    Potree2Provider provider;
    QString error;
    std::shared_ptr<PointCloudDataset> dataset = provider.openMetadata(path, &error);
    if (!dataset) {
        QMessageBox::critical(this, tr("Open Potree Dataset Failed"), error);
        return;
    }

    if (dataset->encoding == QLatin1String("DEFAULT")) {
        std::shared_ptr<PointCloudNodeData> rootData = provider.loadNodeData(
            *dataset,
            dataset->root.get(),
            &error);
        if (!rootData) {
            QMessageBox::critical(this, tr("Open Potree Point Data Failed"), error);
            return;
        }

        if (!m_viewWidget->loadPotreeNode(*dataset, *rootData, &error)) {
            QMessageBox::critical(this, tr("Display Potree Point Data Failed"), error);
            return;
        }
    }

    m_potreeMetadataDataset = std::move(dataset);
    const QString summary = potreeSummaryText(*m_potreeMetadataDataset);
    qInfo().noquote() << summary;
    statusBar()->showMessage(tr("Loaded Potree metadata: %1 records")
                                 .arg(m_potreeMetadataDataset->hierarchyRecordsLoaded),
                             8000);
    QMessageBox::information(this, tr("Potree Metadata Summary"), summary);
}

QString MainWindow::selectPotreeMetadataPath()
{
    QMessageBox sourceDialog(this);
    sourceDialog.setWindowTitle(tr("Open Potree Dataset"));
    sourceDialog.setText(tr("Choose a Potree dataset folder or metadata.json."));
    QPushButton* folderButton = sourceDialog.addButton(tr("Folder"), QMessageBox::AcceptRole);
    QPushButton* fileButton = sourceDialog.addButton(tr("metadata.json"), QMessageBox::AcceptRole);
    sourceDialog.addButton(QMessageBox::Cancel);
    sourceDialog.exec();

    if (sourceDialog.clickedButton() == folderButton) {
        return QFileDialog::getExistingDirectory(this, tr("Open Potree Dataset Folder"));
    }

    if (sourceDialog.clickedButton() == fileButton) {
        return QFileDialog::getOpenFileName(
            this,
            tr("Open Potree Metadata"),
            QString(),
            tr("Potree metadata (metadata.json);;All files (*.*)"));
    }

    return QString();
}

void MainWindow::closePointCloud()
{
    m_viewWidget->clearPointCloud();
    m_potreeMetadataDataset.reset();
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

QString MainWindow::potreeSummaryText(const PointCloudDataset& dataset) const
{
    const QLocale locale;
    QStringList lines;
    lines << tr("Format: %1").arg(dataset.format)
          << tr("Name: %1").arg(dataset.name)
          << tr("Encoding: %1").arg(dataset.encoding)
          << tr("Total points: %1").arg(locale.toString(static_cast<qulonglong>(dataset.totalPoints)))
          << tr("Hierarchy records loaded: %1")
                 .arg(locale.toString(static_cast<qulonglong>(dataset.hierarchyRecordsLoaded)))
          << tr("Max loaded level: %1").arg(dataset.maxLoadedLevel)
          << tr("Proxy nodes: %1").arg(locale.toString(static_cast<qulonglong>(dataset.proxyNodeCount)))
          << tr("First chunk point sum: %1")
                 .arg(locale.toString(static_cast<qulonglong>(dataset.firstChunkPointCount)))
          << tr("Root bounds: [%1] - [%2]").arg(formatVec3(dataset.bounds.min), formatVec3(dataset.bounds.max))
          << tr("Point record size: %1 bytes").arg(dataset.attributes.pointRecordSizeBytes())
          << tr("Attributes: %1").arg(dataset.attributeNames().join(QStringLiteral(", ")));

    if (dataset.root && dataset.root->data) {
        lines << tr("Displayed root points: %1")
                     .arg(locale.toString(static_cast<qulonglong>(dataset.root->data->positions.size())));
    }

    if (dataset.encoding == QLatin1String("BROTLI")) {
        lines << tr("Point data decoding is not supported yet.");
    }

    return lines.join(QLatin1Char('\n'));
}
