#include <QApplication>

#include "SimViewer.h"
#include "MainWindow.h"

#include <json/json.h>

// SA: Need to do this, otherwise the "slots" Qt macro collides with function names in libtorch
#undef slots
  #include <torch/torch.h>
#define slots Q_SLOTS
#include <fstream>
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Set the core profile and version of OpenGL shaders.
    QSurfaceFormat fmt;
    fmt.setVersion(4, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(32);
    fmt.setSamples(0);
    QSurfaceFormat::setDefaultFormat(fmt);

    MainWindow w;

    // Instantiate the cloth viewer.
    SimViewer v;
    w.addViewer(&v);
    w.show();

    return a.exec();
}
