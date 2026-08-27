#pragma once

#include "pointcloud/PointCloudProvider.h"
#include "pointcloud/HierarchyPatch.h"
#include "pointcloud/NodeLoadRequest.h"
#include "pointcloud/NodeLoadResult.h"
#include "pointcloud/PointCloudNodeData.h"
#include "pointcloud/PotreeMetadata.h"

#include <QString>

#include <memory>

// Potree 2.x 文件格式适配器。
//
// openMetadata() 建立首个 Octree 快照；异步阶段的 ensureNodeReady() 只使用
// NodeLoadRequest 读取 hierarchy.bin/octree.bin 并返回 NodeLoadResult。
// applyHierarchyPatch() 必须由 PointCloudRuntime 在主线程调用。
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
    // Proxy：解析 hierarchy patch，再用 patch 根记录给出的点范围解码点数据；
    // Normal/Leaf：直接解码点数据。函数不修改 live Octree 或 OSG。
    NodeLoadResult ensureNodeReady(const PointCloudDataset& dataset,
                                   const NodeLoadRequest& request) const;
    bool loadHierarchyPatch(const PointCloudDataset& dataset,
                            const OctreeNode& proxyNode,
                            HierarchyPatch* patch,
                            QString* errorMessage) const;
    // 将值形式 patch 合并到 live Octree；只允许在主/update线程调用。
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
