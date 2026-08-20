#pragma once

#include <osg/Matrixd>
#include <osg/Vec3d>

struct CameraState {
    osg::Matrixd viewMatrix;
    osg::Matrixd projectionMatrix;
    osg::Vec3d position;
    int viewportHeight = 1;
};
