#include "ui/MainWindow.h"

#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char* argv[])
{
    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(0);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);
    QApplication::setApplicationName("PotreeOSGViewer");
    QApplication::setOrganizationName("PotreeOSGViewer");

    MainWindow window;
    window.resize(1280, 800);
    window.show();

    return app.exec();
}
