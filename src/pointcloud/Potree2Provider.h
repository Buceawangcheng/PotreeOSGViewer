#pragma once

#include "pointcloud/PointCloudProvider.h"
#include "pointcloud/HierarchyPatch.h"
#include "pointcloud/NodeLoadRequest.h"
#include "pointcloud/NodeLoadResult.h"
#include "pointcloud/PointCloudNodeData.h"
#include "pointcloud/PotreeMetadata.h"

#include <QString>

#include <memory>

class Potree2Provider : public PointCloudProvider {
public:
    bool canOpen(const QString& path) const override;
    std::shared_ptr<PointCloudDataset> openMetadata(const QString& path,
                                                    QString* errorMessage) override;
    std::shared_ptr<PointCloudNodeData> loadNodeData(const PointCloudDataset& dataset,
                                                     OctreeNode* node,
                                                     QString* errorMessage) const;
    std::shared_ptr<PointCloudNodeData> loadNodeData(const PointCloudDataset& dataset,
                                                     const NodeLoadRequest& request,
                                                     QString* errorMessage) const;
    NodeLoadResult ensureNodeReady(const PointCloudDataset& dataset,
                                   const NodeLoadRequest& request) const;
    bool loadHierarchyPatch(const PointCloudDataset& dataset,
                            const OctreeNode& proxyNode,
                            HierarchyPatch* patch,
                            QString* errorMessage) const;
    bool applyHierarchyPatch(PointCloudDataset* dataset,
                             const HierarchyPatch& patch,
                             QString* errorMessage) const;

private:
    bool resolveDatasetPaths(const QString& path,
                             QString* metadataPath,
                             QString* datasetDir,
                             QString* errorMessage) const;
    bool readMetadata(const QString& metadataPath,
                      const QString& datasetDir,
                      PotreeMetadata* metadata,
                      QString* errorMessage) const;
    bool parseFirstHierarchyChunk(const PotreeMetadata& metadata,
                                  PointCloudDataset* dataset,
                                  QString* errorMessage) const;
};
