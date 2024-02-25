#-------------------------------------------------
#
# Project created by QtCreator
#
#-------------------------------------------------

QT       += core gui xml opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += openglwidgets widgets

TARGET = rigidBodySim
TEMPLATE = app


SOURCES += main.cpp \
           3rdParty/QGLViewer/camera.cpp \
           3rdParty/QGLViewer/constraint.cpp \
           3rdParty/QGLViewer/frame.cpp \
           3rdParty/QGLViewer/keyFrameInterpolator.cpp \
           3rdParty/QGLViewer/manipulatedCameraFrame.cpp \
           3rdParty/QGLViewer/manipulatedFrame.cpp \
           3rdParty/QGLViewer/mouseGrabber.cpp \
           3rdParty/QGLViewer/qglviewer.cpp \
           3rdParty/QGLViewer/quaternion.cpp \
           3rdParty/QGLViewer/saveSnapshot.cpp \
           3rdParty/QGLViewer/vec.cpp \
           src/Contact.cpp \
           src/Distance.cpp \
           src/EllipseWidget.cpp \
           src/FBOWidget.cpp \
           src/Joint.cpp \
           src/MatrixFreePGS.cpp \
           src/MeshAssets.cpp \
           src/RigidBody.cpp \
           src/RigidBodyRenderer.cpp \
           src/RigidBodySystem.cpp \
           src/SimViewer.cpp \
           src/MainWindow.cpp \
           src/OBJLoader.cpp  \
           src/SimulationSerializer.cpp \
           src/Spherical.cpp
		   
HEADERS  += 3rdParty/QGLViewer/camera.h \
            3rdParty/QGLViewer/config.h \
            3rdParty/QGLViewer/constraint.h \
            3rdParty/QGLViewer/domUtils.h \
            3rdParty/QGLViewer/frame.h \
            3rdParty/QGLViewer/keyFrameInterpolator.h \
            3rdParty/QGLViewer/manipulatedCameraFrame.h \
            3rdParty/QGLViewer/manipulatedFrame.h \
            3rdParty/QGLViewer/mouseGrabber.h \
            3rdParty/QGLViewer/qglviewer.h \
            3rdParty/QGLViewer/quaternion.h \
            3rdParty/QGLViewer/vec.h \
    include/CollisionUtils.hpp \
    include/Contact.h \
    include/Distance.h \
    include/EllipseWidget.h \
    include/FBOWidget.h \
    include/Geometry.hpp \
    include/Joint.h \
    include/MatrixFreePGS.h \
    include/MeshAssets.h \
    include/RigidBody.h \
    include/RigidBodyRenderer.h \
    include/RigidBodySystem.h \
    include/Scenarios.hpp \
    include/ShaderVars.h \
    include/SimViewer.h \
    include/MainWindow.h \
    include/OBJLoader.h  \
    include/SimulationSerializer.h \
    include/Spherical.h

INCLUDEPATH += 3rdParty/Eigen3/include/eigen3 \
               3rdParty/QGLViewer \
               3rdParty/jsoncpp/include \
               3rdParty/Discregrid/include \
               include

DEFINES += JSON_DLL
LIBS += -L3rdParty/jsoncpp/lib -L3rdParty/jsoncpp/bin -ljsoncpp
LIBS += -L3rdParty/Discregrid/lib -L3rdParty/Discregrid/bin

CONFIG(debug, debug|release) {
    LIBS += -lDiscregrid_d
}
CONFIG(release, debug|release) {
    LIBS += -lDiscregrid
}

DISTFILES += glsl/basicShader.frag \
             glsl/basicShader.vert \
             glsl/contactTextureShader.frag \
             glsl/contactTextureShader.vert \
             glsl/contactWeightShader.frag \
             glsl/contactWeightShader.vert \
             glsl/depthShader.frag \
             glsl/depthShader.vert \
             glsl/spring.frag \
             glsl/spring.vert

FORMS += 3rdParty/QGLViewer/ImageInterface.ui \
         3rdParty/QGLViewer/VRenderInterface.ui \
         mainwindow.ui \
         mainwindow.ui

CONFIG *= c++11 debug_and_release console qt opengl warn_on thread create_prl rtti

DEFINES *= QGLVIEWER_STATIC
win32 {
    DEFINES *= NOMINMAX _USE_MATH_DEFINES
    QMAKE_CXXFLAGS += -openmp
    #QMAKE_CXXFLAGS += -arch:AVX
    QMAKE_CXXFLAGS += -D "_CRT_SECURE_NO_WARNINGS"
    QMAKE_CXXFLAGS_RELEASE *= -O2
}

win32 {
  contains ( QT_MAJOR_VERSION, 5 ) {
    greaterThan( QT_MINOR_VERSION, 4) {
      LIBS *= -lopengl32
    }
  }
}

SUBDIRS += \
    3rdParty/QGLViewer/QGLViewer.pro
