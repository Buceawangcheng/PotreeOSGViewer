#pragma once

#include "pointcloud/BoundingBox.h"
#include "pointcloud/OctreeNode.h"
#include "pointcloud/PointAttributes.h"

#include <osg/Vec3d>

#include <QString>
#include <QStringList>

#include <cstdint>
#include <memory>

class PointCloudDataset {
public:
    QString sourcePath;
    QString name;
    QString format;
    QString version;
    QString encoding;
    std::uint64_t totalPoints = 0;
    double spacing = 0.0;
    osg::Vec3d offset;
    osg::Vec3d scale;
    BoundingBox bounds;
    PointAttributes attributes;
    std::unique_ptr<OctreeNode> root;

    std::uint64_t hierarchyRecordsLoaded = 0;
    std::uint32_t hierarchyDepth = 0;
    std::uint32_t maxLoadedLevel = 0;
    std::uint64_t firstChunkPointCount = 0;
    std::uint64_t proxyNodeCount = 0;

    QStringList attributeNames() const
    {
        QStringList names;
        for (const PointAttribute& attribute : attributes.items()) {
            names.append(QString::fromStdString(attribute.name));
        }

        return names;
    }
};
