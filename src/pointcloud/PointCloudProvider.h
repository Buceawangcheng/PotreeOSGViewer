#pragma once

#include "pointcloud/PointCloudDataset.h"

#include <QString>

#include <memory>

class PointCloudProvider {
public:
    virtual ~PointCloudProvider() = default;

    virtual bool canOpen(const QString& path) const = 0;
    virtual std::shared_ptr<PointCloudDataset> openMetadata(const QString& path,
                                                    QString* errorMessage) = 0;
};
