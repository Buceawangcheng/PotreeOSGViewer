#pragma once

#include "pointcloud/BoundingBox.h"
#include "pointcloud/OctreeNode.h"
#include "pointcloud/PointAttributes.h"

#include <osg/Vec3d>

#include <QString>
#include <QStringList>

#include <cstdint>
#include <memory>

// 一个已打开的 Potree 数据集及其 live Octree 所有者。
// metadata 字段在打开时建立；root 以下的 Proxy 节点会随着主线程应用
// HierarchyPatch 逐步扩展。工作线程通过 shared_ptr 保证文件元数据生命周期，
// 但不得直接修改 root 或节点状态。
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
    std::unique_ptr<OctreeNode> root; // 主/update线程拥有和扩展的运行时树根。

    // 已应用 hierarchy 分块的累计统计，由主线程更新。
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
