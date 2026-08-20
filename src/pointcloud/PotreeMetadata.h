#pragma once

#include "pointcloud/BoundingBox.h"
#include "pointcloud/PointAttributes.h"

#include <osg/Vec3d>

#include <QString>

#include <cstdint>

struct PotreeHierarchyMetadata {
    std::uint64_t firstChunkSize = 0;
    std::uint32_t stepSize = 0;
    std::uint32_t depth = 0;
};

struct PotreeMetadata {
    QString metadataPath;
    QString datasetDir;
    QString version;
    QString name;
    QString encoding;
    std::uint64_t points = 0;
    PotreeHierarchyMetadata hierarchy;
    osg::Vec3d offset;
    osg::Vec3d scale;
    double spacing = 0.0;
    BoundingBox bounds;
    PointAttributes attributes;
};
