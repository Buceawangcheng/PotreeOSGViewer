#pragma once

#include "pointcloud/PointCloudDataset.h"

#include <memory>
#include <utility>

class PointCloudLayer {
public:
    explicit PointCloudLayer(std::shared_ptr<PointCloudDataset> dataset)
        : m_dataset(std::move(dataset))
    {
    }

    const PointCloudDataset& dataset() const
    {
        return *m_dataset;
    }

private:
    std::shared_ptr<PointCloudDataset> m_dataset;
};
