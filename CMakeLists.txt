cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
    file(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}" VCPKG_ROOT_DIR)
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT_DIR}/scripts/buildsystems/vcpkg.cmake"
        CACHE FILEPATH "vcpkg toolchain file")
endif()

project(PotreeOSGViewer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(QT_ROOT_DIR "" CACHE PATH "Qt 5 MSVC x64 root")
set(OSGQT_ROOT "" CACHE PATH "osgQt install root containing debug and release")
set(OSGQT_DEBUG_ROOT "" CACHE PATH "osgQt Debug install root")
set(OSGQT_RELEASE_ROOT "" CACHE PATH "osgQt Release install root")
set(VCPKG_INSTALLED_DIR "" CACHE PATH "vcpkg installed root")
set(OSG_PLUGIN_VERSION "3.6.5" CACHE STRING "OpenSceneGraph plugin directory version")
set(OSG_PLUGINS_DEBUG "" CACHE PATH "OpenSceneGraph Debug plugin directory")
set(OSG_PLUGINS_RELEASE "" CACHE PATH "OpenSceneGraph Release plugin directory")

if(QT_ROOT_DIR)
    list(PREPEND CMAKE_PREFIX_PATH "${QT_ROOT_DIR}")
endif()

if(OSGQT_ROOT)
    if(NOT OSGQT_DEBUG_ROOT)
        set(OSGQT_DEBUG_ROOT "${OSGQT_ROOT}/debug")
    endif()
    if(NOT OSGQT_RELEASE_ROOT)
        set(OSGQT_RELEASE_ROOT "${OSGQT_ROOT}/release")
    endif()
endif()

if(NOT OSGQT_DEBUG_ROOT OR NOT OSGQT_RELEASE_ROOT)
    message(FATAL_ERROR
        "Set OSGQT_ROOT, or set both OSGQT_DEBUG_ROOT and OSGQT_RELEASE_ROOT.")
endif()

find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(unofficial-osg CONFIG REQUIRED)

if(NOT OSG_INCLUDE_DIR)
    unset(OSG_INCLUDE_DIR CACHE)
    find_path(OSG_INCLUDE_DIR
        NAMES osg/Node
        DOC "OpenSceneGraph include root"
        REQUIRED
    )
endif()

if(NOT QT_ROOT_DIR)
    get_target_property(QT_QMAKE_EXECUTABLE Qt5::qmake IMPORTED_LOCATION)
    if(NOT QT_QMAKE_EXECUTABLE)
        message(FATAL_ERROR
            "Qt was found, but QT_ROOT_DIR could not be inferred. Set QT_ROOT_DIR explicitly.")
    endif()
    get_filename_component(QT_BIN_DIR "${QT_QMAKE_EXECUTABLE}" DIRECTORY)
    get_filename_component(QT_ROOT_DIR "${QT_BIN_DIR}" DIRECTORY)
else()
    set(QT_BIN_DIR "${QT_ROOT_DIR}/bin")
endif()

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
    resources/resources.qrc
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
    src/viewer/camera/CameraMath.cpp
    src/viewer/camera/CameraMath.h
    src/viewer/camera/CesiumCameraManipulator.cpp
    src/viewer/camera/CesiumCameraManipulator.h
    src/viewer/camera/DepthBufferPicker.cpp
    src/viewer/camera/DepthBufferPicker.h
    src/viewer/camera/PickDebugVisualizer.cpp
    src/viewer/camera/PickDebugVisualizer.h
    src/viewer/PotreeColorMode.h
    src/viewer/PointCloudShaderState.cpp
    src/viewer/PointCloudShaderState.h
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
    "${OSG_INCLUDE_DIR}"
)

include(CTest)

if(BUILD_TESTING)
    add_executable(PotreePointCloudTests
        resources/resources.qrc
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
        src/viewer/camera/CameraMath.cpp
        src/viewer/camera/CameraMath.h
        src/viewer/camera/CesiumCameraManipulator.cpp
        src/viewer/camera/CesiumCameraManipulator.h
        src/viewer/camera/DepthBufferPicker.cpp
        src/viewer/camera/DepthBufferPicker.h
        src/viewer/camera/PickDebugVisualizer.cpp
        src/viewer/camera/PickDebugVisualizer.h
        src/viewer/PotreeColorMode.h
        src/viewer/PointCloudShaderState.cpp
        src/viewer/PointCloudShaderState.h
        src/viewer/PotreeRenderMasks.h
        src/viewer/PotreeSceneBackend.cpp
        src/viewer/PotreeSceneBackend.h
    )

    target_link_libraries(PotreePointCloudTests PRIVATE
        Qt5::Core
        Qt5::Gui
        unofficial::osg::osg
        unofficial::osg::osgDB
        unofficial::osg::osgGA
    )

    target_include_directories(PotreePointCloudTests PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${OSG_INCLUDE_DIR}"
    )

    if(MSVC)
        target_compile_options(PotreePointCloudTests PRIVATE /W4 /permissive- /utf-8)
    endif()

    add_test(NAME PotreePointCloudTests COMMAND PotreePointCloudTests)
endif()

if(MSVC)
    target_compile_options(PotreeOSGViewer PRIVATE /W4 /permissive- /utf-8)
endif()

if(NOT VCPKG_INSTALLED_DIR)
    if(DEFINED _VCPKG_INSTALLED_DIR)
        set(VCPKG_INSTALLED_DIR "${_VCPKG_INSTALLED_DIR}")
    elseif(DEFINED VCPKG_ROOT_DIR)
        set(VCPKG_INSTALLED_DIR "${VCPKG_ROOT_DIR}/installed")
    elseif(DEFINED ENV{VCPKG_ROOT})
        file(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}/installed" VCPKG_INSTALLED_DIR)
    endif()
endif()

if(VCPKG_TARGET_TRIPLET)
    set(POTREEOSGVIEWER_VCPKG_TRIPLET "${VCPKG_TARGET_TRIPLET}")
else()
    set(POTREEOSGVIEWER_VCPKG_TRIPLET "x64-windows")
endif()

if(VCPKG_INSTALLED_DIR)
    set(VCPKG_TRIPLET_DIR
        "${VCPKG_INSTALLED_DIR}/${POTREEOSGVIEWER_VCPKG_TRIPLET}")
    set(VCPKG_BIN_RELEASE "${VCPKG_TRIPLET_DIR}/bin")
    set(VCPKG_BIN_DEBUG "${VCPKG_TRIPLET_DIR}/debug/bin")
endif()

if(NOT OSG_PLUGINS_RELEASE)
    foreach(candidate
            "${VCPKG_TRIPLET_DIR}/plugins/osgPlugins-${OSG_PLUGIN_VERSION}"
            "${OSGQT_RELEASE_ROOT}/share/OpenSceneGraph/bin/osgPlugins-${OSG_PLUGIN_VERSION}")
        if(EXISTS "${candidate}")
            set(OSG_PLUGINS_RELEASE "${candidate}")
            break()
        endif()
    endforeach()
endif()

if(NOT OSG_PLUGINS_DEBUG)
    foreach(candidate
            "${VCPKG_TRIPLET_DIR}/debug/plugins/osgPlugins-${OSG_PLUGIN_VERSION}"
            "${OSGQT_DEBUG_ROOT}/share/OpenSceneGraph/bin/osgPlugins-${OSG_PLUGIN_VERSION}")
        if(EXISTS "${candidate}")
            set(OSG_PLUGINS_DEBUG "${candidate}")
            break()
        endif()
    endforeach()
endif()

if(NOT IS_DIRECTORY "${OSG_PLUGINS_RELEASE}"
        OR NOT IS_DIRECTORY "${OSG_PLUGINS_DEBUG}")
    message(FATAL_ERROR
        "OpenSceneGraph plugin directories were not found. Set OSG_PLUGINS_RELEASE and OSG_PLUGINS_DEBUG.")
endif()

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
        "$<TARGET_FILE_DIR:PotreeOSGViewer>/osgPlugins-${OSG_PLUGIN_VERSION}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE"
        "${CMAKE_CURRENT_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
        "$<TARGET_FILE_DIR:PotreeOSGViewer>"
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party_licenses"
        "$<TARGET_FILE_DIR:PotreeOSGViewer>/third_party_licenses"
    COMMENT "Deploying runtime files and license notices"
)

message(STATUS "Qt root: ${QT_ROOT_DIR}")
message(STATUS "vcpkg installed: ${VCPKG_INSTALLED_DIR}")
message(STATUS "OSG release plugins: ${OSG_PLUGINS_RELEASE}")
message(STATUS "OSG debug plugins: ${OSG_PLUGINS_DEBUG}")
message(STATUS "osgQt include: ${OSGQOPENGL_INCLUDE_DIR}")
message(STATUS "osgQt release lib: ${OSGQOPENGL_LIBRARY_RELEASE}")
message(STATUS "osgQt debug lib: ${OSGQOPENGL_LIBRARY_DEBUG}")
