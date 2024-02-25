cmake_minimum_required(VERSION 3.18)

# Add folder where are supportive functions
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
project(textureFriction)

set(QT_MAJOR_VERSION 5)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#See if the environment var is set
if(DEFINED ENV{Qt5_DIR})
    message(STATUS "Looking for Qt in: " $ENV{Qt5_DIR})
else()
    message(STATUS "Qt5_DIR environment variable not set. Checking default paths.")
endif()

if(DEFINED ENV{Torch_DIR})
	message(STATUS "Looking for libtorch in: " $ENV{Torch_DIR})
else()
	set(Torch_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/libtorch/share/cmake/Torch)
	message(STATUS "Torch_DIR environment variable not set. Checking default paths.")
endif()

include(cmake/FindDiscregrid.cmake)
include(cmake/FindJsonCpp.cmake)

find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED PATHS $ENV{Qt5_DIR})
find_package(Qt5 COMPONENTS Widgets REQUIRED PATHS $ENV{Qt5_DIR})
find_package(Qt5 COMPONENTS Xml REQUIRED PATHS $ENV{Qt5_DIR})
find_package(Qt5 COMPONENTS OpenGL REQUIRED PATHS $ENV{Qt5_DIR})
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


set(textureFriction_HEADERS include/CollisionUtils.hpp
			include/Contact.h
			include/Distance.h
			include/EllipseWidget.h
			include/FBOWidget.h
			include/Geometry.hpp
			include/Joint.h 
			include/MainWindow.h
			include/MatrixFreePGS.h 
			include/MeshAssets.h 
			include/OBJLoader.h 
			include/RigidBody.h 
			include/RigidBodyRenderer.h 
			include/RigidBodySystem.h 
			include/Scenarios.hpp 
			include/ShaderVars.h 
			include/SimViewer.h 
			include/SimulationSerializer.h 
			include/Spherical.h )
set(textureFriction_SOURCE src/Contact.cpp 
		   src/Distance.cpp
		   src/EllipseWidget.cpp
		   src/FBOWidget.cpp
		   src/Joint.cpp
		   src/MainWindow.cpp
		   src/MatrixFreePGS.cpp
		   src/MeshAssets.cpp
		   src/OBJLoader.cpp
		   src/RigidBody.cpp
		   src/RigidBodyRenderer.cpp
		   src/RigidBodySystem.cpp
		   src/SimViewer.cpp
		   src/SimulationSerializer.cpp
		   src/Spherical.cpp ) 

set(qglviewer_HEADERS 3rdParty/QGLViewer/camera.h 
            3rdParty/QGLViewer/config.h 
            3rdParty/QGLViewer/constraint.h 
            3rdParty/QGLViewer/domUtils.h 
            3rdParty/QGLViewer/frame.h 
            3rdParty/QGLViewer/keyFrameInterpolator.h 
            3rdParty/QGLViewer/manipulatedCameraFrame.h 
            3rdParty/QGLViewer/manipulatedFrame.h 
            3rdParty/QGLViewer/mouseGrabber.h 
            3rdParty/QGLViewer/qglviewer.h 
            3rdParty/QGLViewer/quaternion.h 
            3rdParty/QGLViewer/vec.h )
set(qglviewer_SOURCE 3rdParty/QGLViewer/camera.cpp 
           3rdParty/QGLViewer/constraint.cpp 
           3rdParty/QGLViewer/frame.cpp 
           3rdParty/QGLViewer/keyFrameInterpolator.cpp 
           3rdParty/QGLViewer/manipulatedCameraFrame.cpp 
           3rdParty/QGLViewer/manipulatedFrame.cpp 
           3rdParty/QGLViewer/mouseGrabber.cpp 
           3rdParty/QGLViewer/qglviewer.cpp 
           3rdParty/QGLViewer/quaternion.cpp 
           3rdParty/QGLViewer/saveSnapshot.cpp 
           3rdParty/QGLViewer/vec.cpp )
		   
set(shaders_SOURCE glsl/basicShader.frag 
             glsl/basicShader.vert 
             glsl/contactTextureShader.frag 
             glsl/contactTextureShader.vert 
             glsl/contactWeightShader.frag 
             glsl/contactWeightShader.vert 
             glsl/depthShader.frag 
             glsl/depthShader.vert 
             glsl/spring.frag 
             glsl/spring.vert ) 
			 
set(forms_UI src/mainwindow.ui 3rdParty/QGLViewer/ImageInterface.ui 3rdParty/QGLViewer/VRenderInterface.ui)

        
add_executable(textureFriction main.cpp
          ${textureFriction_HEADERS} ${textureFriction_SOURCE} 
          ${qglviewer_HEADERS} ${qglviewer_SOURCE} ${forms_UI})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/QGLViewer ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/jsoncpp/include ${COMMON_INCLUDES})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${DISCREGRID_INCLUDE_DIRS})
include_directories(${JSONCPP_INCLUDE_DIRS})
include_directories(${TORCH_INCLUDE_DIRS})
target_link_libraries(textureFriction general Eigen3::Eigen Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Xml OpenGL::GL OpenMP::OpenMP_CXX ${TORCH_LIBRARIES})
target_link_libraries(textureFriction debug ${DISCREGRID_LIBRARY_DEBUG} ${JSONCPP_LIBRARY_DEBUG})
target_link_libraries(textureFriction optimized ${DISCREGRID_LIBRARY_RELEASE} ${JSONCPP_LIBRARY_RELEASE})


add_definitions(-DJSON_DLL) 
add_definitions(-DQGLVIEWER_STATIC)
add_definitions(-DNOMINMAX)
add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# Setup source code groups
#
source_group(src FILES main.cpp ${textureFriction_SOURCE})
source_group(include FILES ${textureFriction_HEADERS})
source_group(qglviewer/src FILES ${qglviewer_SOURCE})
source_group(qglviewer/include FILES ${qglviewer_HEADERS})
source_group(glsl FILES ${shaders_SOURCE})

set_property(TARGET textureFriction PROPERTY CXX_STANDARD 17)

if(MSVC) 
	set_property(TARGET textureFriction PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
	
	# The following code block is suggested to be used on Windows.
	# According to https://github.com/pytorch/pytorch/issues/25457,
	# the DLLs need to be copied to avoid memory errors.
	file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
	add_custom_command(TARGET textureFriction
					 POST_BUILD
					 COMMAND ${CMAKE_COMMAND} -E copy_if_different
					 ${TORCH_DLLS}
					 $<TARGET_FILE_DIR:textureFriction>)
endif (MSVC)
