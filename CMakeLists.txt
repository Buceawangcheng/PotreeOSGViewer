cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "D:/wc/vcpkg/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE FILEPATH "vcpkg toolchain file")
endif()

project(PotreeOSGViewer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(QT_ROOT_DIR "D:/Qt/MyQt/5.15.2/msvc2019_64" CACHE PATH "Qt 5.15.2 MSVC2019 x64 root")
set(OSGQT_ROOT "D:/wc/software/AI/osg/osgQt-master/install" CACHE PATH "osgQt install root")
set(OSGQT_DEBUG_ROOT "${OSGQT_ROOT}/debug" CACHE PATH "osgQt Debug install root")
set(OSGQT_RELEASE_ROOT "${OSGQT_ROOT}/release" CACHE PATH "osgQt Release install root")
set(VCPKG_INSTALLED_DIR "D:/wc/vcpkg/vcpkg/installed" CACHE PATH "vcpkg installed root")

list(APPEND CMAKE_PREFIX_PATH "${QT_ROOT_DIR}")
set(Qt5_DIR "${QT_ROOT_DIR}/lib/cmake/Qt5" CACHE PATH "Qt5 CMake package directory" FORCE)
set(Qt5Core_DIR "${QT_ROOT_DIR}/lib/cmake/Qt5Core" CACHE PATH "Qt5Core CMake package directory" FORCE)
set(Qt5Gui_DIR "${QT_ROOT_DIR}/lib/cmake/Qt5Gui" CACHE PATH "Qt5Gui CMake package directory" FORCE)
set(Qt5Widgets_DIR "${QT_ROOT_DIR}/lib/cmake/Qt5Widgets" CACHE PATH "Qt5Widgets CMake package directory" FORCE)
set(Qt5OpenGL_DIR "${QT_ROOT_DIR}/lib/cmake/Qt5OpenGL" CACHE PATH "Qt5OpenGL CMake package directory" FORCE)

find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(unofficial-osg CONFIG REQUIRED)

find_path(OSGQOPENGL_INCLUDE_DIR
    NAMES osgQOpenGL/osgQOpenGLWidget
    PATHS
        "${OSGQT_RELEASE_ROOT}/include"
        "${OSGQT_DEBUG_ROOT}/include"
    NO_DEFAULT_PATH
    REQUIRED
)

find_library(OSGQOPENGL_LIBRARY_RELEASE
    NAMES osgQOpenGL
    PATHS "${OSGQT_RELEASE_ROOT}/lib"
    NO_DEFAULT_PATH
    REQUIRED
)

find_library(OSGQOPENGL_LIBRARY_DEBUG
    NAMES osgQOpenGLd osgQOpenGL
    PATHS "${OSGQT_DEBUG_ROOT}/lib"
    NO_DEFAULT_PATH
    REQUIRED
)

find_file(OSGQOPENGL_DLL_RELEASE
    NAMES osg145-osgQOpenGL.dll osgQOpenGL.dll
    PATHS "${OSGQT_RELEASE_ROOT}/bin"
    NO_DEFAULT_PATH
    REQUIRED
)

find_file(OSGQOPENGL_DLL_DEBUG
    NAMES osg145-osgQOpenGLd.dll osgQOpenGLd.dll
    PATHS "${OSGQT_DEBUG_ROOT}/bin"
    NO_DEFAULT_PATH
    REQUIRED
)

find_file(QT_QWINDOWS_PLUGIN_RELEASE
    NAMES qwindows.dll
    PATHS "${QT_ROOT_DIR}/plugins/platforms"
    NO_DEFAULT_PATH
    REQUIRED
)

find_file(QT_QWINDOWS_PLUGIN_DEBUG
    NAMES qwindowsd.dll qwindows.dll
    PATHS "${QT_ROOT_DIR}/plugins/platforms"
    NO_DEFAULT_PATH
    REQUIRED
)

add_library(osgQOpenGL::osgQOpenGL SHARED IMPORTED GLOBAL)
set_target_properties(osgQOpenGL::osgQOpenGL PROPERTIES
    IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
    IMPORTED_IMPLIB_DEBUG "${OSGQOPENGL_LIBRARY_DEBUG}"
    IMPORTED_LOCATION_DEBUG "${OSGQOPENGL_DLL_DEBUG}"
    IMPORTED_IMPLIB_RELEASE "${OSGQOPENGL_LIBRARY_RELEASE}"
    IMPORTED_LOCATION_RELEASE "${OSGQOPENGL_DLL_RELEASE}"
    MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE
    MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE
    INTERFACE_INCLUDE_DIRECTORIES "${OSGQOPENGL_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "Qt5::Widgets;Qt5::OpenGL;unofficial::osg::osg;unofficial::osg::osgDB;unofficial::osg::osgGA;unofficial::osg::osgUtil;unofficial::osg::osgViewer;unofficial::osg::OpenThreads;opengl32"
)

add_executable(PotreeOSGViewer
    src/main.cpp
    src/pointcloud/BoundingBox.h
    src/pointcloud/CameraState.h
    src/pointcloud/HierarchyPatch.h
    src/pointcloud/LodSelector.cpp
    src/pointcloud/LodSelector.h
    src/pointcloud/NodeLoadRequest.h
    src/pointcloud/NodeLoadResult.h
    src/pointcloud/NodeLoadScheduler.cpp
    src/pointcloud/NodeLoadScheduler.h
    src/pointcloud/OctreeNode.h
    src/pointcloud/PointCloudRuntime.cpp
    src/pointcloud/PointCloudRuntime.h
    src/pointcloud/PointCloudUpdateCallback.cpp
    src/pointcloud/PointCloudUpdateCallback.h
    src/pointcloud/PointAttributes.h
    src/pointcloud/PointCloudDataset.h
    src/pointcloud/PointCloudLayer.h
    src/pointcloud/PointCloudNodeData.h
    src/pointcloud/PointCloudProvider.h
    src/pointcloud/PotreeMetadata.h
    src/pointcloud/Potree2Provider.cpp
    src/pointcloud/Potree2Provider.h
    src/ui/MainWindow.cpp
    src/ui/MainWindow.h
    src/viewer/OsgViewWidget.cpp
    src/viewer/OsgViewWidget.h
    src/viewer/PotreeColorMode.h
    src/viewer/PotreeRenderMasks.h
    src/viewer/PotreeSceneBackend.cpp
    src/viewer/PotreeSceneBackend.h
    src/viewer/SceneManager.cpp
    src/viewer/SceneManager.h
)

target_link_libraries(PotreeOSGViewer PRIVATE
    Qt5::Widgets
    Qt5::OpenGL
    osgQOpenGL::osgQOpenGL
    unofficial::osg::osg
    unofficial::osg::osgDB
    unofficial::osg::osgGA
    unofficial::osg::osgUtil
    unofficial::osg::osgViewer
    unofficial::osg::OpenThreads
)

target_include_directories(PotreeOSGViewer PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

include(CTest)

if(BUILD_TESTING)
    add_executable(PotreePointCloudTests
        tests/PointCloudTests.cpp
        src/pointcloud/BoundingBox.h
        src/pointcloud/CameraState.h
        src/pointcloud/HierarchyPatch.h
        src/pointcloud/LodSelector.cpp
        src/pointcloud/LodSelector.h
        src/pointcloud/NodeLoadRequest.h
        src/pointcloud/NodeLoadResult.h
        src/pointcloud/NodeLoadScheduler.cpp
        src/pointcloud/NodeLoadScheduler.h
        src/pointcloud/OctreeNode.h
        src/pointcloud/PointAttributes.h
        src/pointcloud/PointCloudDataset.h
        src/pointcloud/PointCloudNodeData.h
        src/pointcloud/PointCloudProvider.h
        src/pointcloud/PotreeMetadata.h
        src/pointcloud/Potree2Provider.cpp
        src/pointcloud/Potree2Provider.h
        src/viewer/SceneManager.cpp
        src/viewer/SceneManager.h
        src/viewer/PotreeColorMode.h
        src/viewer/PotreeRenderMasks.h
        src/viewer/PotreeSceneBackend.cpp
        src/viewer/PotreeSceneBackend.h
    )

    target_link_libraries(PotreePointCloudTests PRIVATE
        Qt5::Core
        unofficial::osg::osg
        unofficial::osg::osgDB
    )

    target_include_directories(PotreePointCloudTests PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${VCPKG_INSTALLED_DIR}/x64-windows/include"
    )

    if(MSVC)
        target_compile_options(PotreePointCloudTests PRIVATE /W4 /permissive-)
    endif()

    add_test(NAME PotreePointCloudTests COMMAND PotreePointCloudTests)
endif()

if(MSVC)
    target_compile_options(PotreeOSGViewer PRIVATE /W4 /permissive-)
endif()

set(QT_BIN_DIR "${QT_ROOT_DIR}/bin")
set(VCPKG_BIN_RELEASE "${VCPKG_INSTALLED_DIR}/x64-windows/bin")
set(VCPKG_BIN_DEBUG "${VCPKG_INSTALLED_DIR}/x64-windows/debug/bin")
set(OSG_PLUGINS_RELEASE "${OSGQT_RELEASE_ROOT}/share/OpenSceneGraph/bin/osgPlugins-3.6.5")
set(OSG_PLUGINS_DEBUG "${VCPKG_INSTALLED_DIR}/x64-windows/debug/plugins/osgPlugins-3.6.5")

set_target_properties(PotreeOSGViewer PROPERTIES
    VS_DEBUGGER_ENVIRONMENT
        "PATH=${QT_BIN_DIR};${OSGQT_RELEASE_ROOT}/bin;${OSGQT_DEBUG_ROOT}/bin;${VCPKG_BIN_RELEASE};${VCPKG_BIN_DEBUG};%PATH%\nOSG_LIBRARY_PATH=$<IF:$<CONFIG:Debug>,${OSG_PLUGINS_DEBUG},${OSG_PLUGINS_RELEASE}>"
)

add_custom_command(TARGET PotreeOSGViewer POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "$<TARGET_FILE:osgQOpenGL::osgQOpenGL>"
        "$<TARGET_FILE_DIR:PotreeOSGViewer>"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "$<TARGET_FILE:Qt5::Core>"
        "$<TARGET_FILE:Qt5::Gui>"
        "$<TARGET_FILE:Qt5::Widgets>"
        "$<TARGET_FILE:Qt5::OpenGL>"
        "$<TARGET_FILE_DIR:PotreeOSGViewer>"
    COMMAND "${CMAKE_COMMAND}" -E make_directory
        "$<TARGET_FILE_DIR:PotreeOSGViewer>/plugins/platforms"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "$<IF:$<CONFIG:Debug>,${QT_QWINDOWS_PLUGIN_DEBUG},${QT_QWINDOWS_PLUGIN_RELEASE}>"
        "$<TARGET_FILE_DIR:PotreeOSGViewer>/plugins/platforms"
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
        "$<IF:$<CONFIG:Debug>,${OSG_PLUGINS_DEBUG},${OSG_PLUGINS_RELEASE}>"
        "$<TARGET_FILE_DIR:PotreeOSGViewer>/osgPlugins-3.6.5"
    COMMENT "Deploying Qt, osgQt and OSG plugin runtime files"
)

message(STATUS "Qt root: ${QT_ROOT_DIR}")
message(STATUS "vcpkg installed: ${VCPKG_INSTALLED_DIR}")
message(STATUS "osgQt include: ${OSGQOPENGL_INCLUDE_DIR}")
message(STATUS "osgQt release lib: ${OSGQOPENGL_LIBRARY_RELEASE}")
message(STATUS "osgQt debug lib: ${OSGQOPENGL_LIBRARY_DEBUG}")
