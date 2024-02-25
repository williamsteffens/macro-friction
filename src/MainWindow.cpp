#include "MainWindow.h"
#include "ui_mainwindow.h"
#include <QBoxLayout>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addViewer(SimViewer* viewer)
{
    // Set the viewer
    QLayout *layout = new QHBoxLayout;
    layout->addWidget(viewer);
    ui->frame->setLayout(layout);

    connect(ui->stepOnce, SIGNAL(clicked()), viewer, SLOT(stepOnce()));
    connect(ui->maxIterations, SIGNAL(valueChanged(int)), viewer, SLOT(setMaxIterations(int)));
    connect(ui->drawMesh, SIGNAL(toggled(bool)), viewer, SLOT(setDrawMesh(bool)));
    connect(ui->drawContacts, SIGNAL(toggled(bool)), viewer, SLOT(setDrawContacts(bool)));
    connect(ui->timeStep, SIGNAL(valueChanged(double)), viewer, SLOT(setTimestep(double)));
    connect(ui->subSteps, SIGNAL(valueChanged(int)), viewer, SLOT(setSubsteps(int)));
    connect(ui->contactStiffness, SIGNAL(valueChanged(double)), viewer, SLOT(setContactStiffness(double)));
    connect(ui->contactDamping, SIGNAL(valueChanged(double)), viewer, SLOT(setContactDamping(double)));
    connect(ui->kA, SIGNAL(valueChanged(double)), viewer, SLOT(setPloughingCoeffA(double)));
    connect(ui->kB, SIGNAL(valueChanged(double)), viewer, SLOT(setPloughingCoeffB(double)));
    connect(ui->frictionCoeff, SIGNAL(valueChanged(double)), viewer, SLOT(setFrictionCoefficient(double)));
    connect(ui->pause, SIGNAL(toggled(bool)), viewer, SLOT(setPaused(bool)));
    connect(ui->contactFrameAlign, SIGNAL(toggled(bool)), viewer, SLOT(setContactFrameAlign(bool)));
    connect(ui->saveFrames, SIGNAL(toggled(bool)), viewer, SLOT(setSaveFrames(bool)));

    connect(ui->patchSize, SIGNAL(valueChanged(double)), viewer, SLOT(setPatchSize(double)));
    connect(ui->patchResolution, SIGNAL(valueChanged(int)), viewer, SLOT(setPatchResolution(int)));

    connect(ui->applyExtImpulse, SIGNAL(clicked()), viewer, SLOT(applyExtImpulse()));
    connect(ui->extForceId, SIGNAL(valueChanged(int)), viewer, SLOT(setExtForceId(int)));
    connect(ui->extForceDir, SIGNAL(currentIndexChanged(int)), viewer, SLOT(setExtForceDir(int)));
    connect(ui->extForceMag, SIGNAL(valueChanged(double)), viewer, SLOT(setExtForceMagnitude(double)));

    connect(ui->boxOnPlaneSawtooth, SIGNAL(clicked()), viewer, SLOT(createBoxOnSawtoothPlane()));
    connect(ui->boxOnRoughPlane, SIGNAL(clicked()), viewer, SLOT(createBoxOnRoughPlane()));
    connect(ui->threeBoxesSliding, SIGNAL(clicked()), viewer, SLOT(createThreeBoxesSliding()));
    connect(ui->twoBoxesAnisotropic, SIGNAL(clicked()), viewer, SLOT(createTwoBoxesAnisotropic()));
    connect(ui->stack, SIGNAL(clicked()), viewer, SLOT(createStack()));
    connect(ui->boxInclinedPlane, SIGNAL(clicked()), viewer, SLOT(createBoxOnInclinedPlane()));
    connect(ui->bolaRough, SIGNAL(clicked()), viewer, SLOT(createBolaRough()));
    connect(ui->bolaSmooth, SIGNAL(clicked()), viewer, SLOT(createBolaSmooth()));
    connect(ui->nutAndBolt, SIGNAL(clicked()), viewer, SLOT(createNutAndBolt()));
    connect(ui->testButton, SIGNAL(clicked()), viewer, SLOT(createTestSystem()));

    // Update status bar message
    connect(viewer, SIGNAL(statusMessageChanged(QString)), ui->statusBar, SLOT(showMessage(QString)));

    viewer->setFrameBufferWidget(ui->fboWidget);
}
