#pragma once

#include "viewer/PotreeColorMode.h"

#include <osg/Group>
#include <osg/Node>
#include <osg/ref_ptr>

#include <QString>

#include <cstdint>
#include <memory>
#include <string>

class PointCloudDataset;
struct BoundingBox;
struct PointCloudNodeData;
class PotreeSceneBackend;

class SceneManager
{
public:
    SceneManager();
    ~SceneManager();

    osg::Group* root() const;

    bool loadPointCloud(const std::string& nativeFilePath,
                        const QString& displayFilePath,
                        float pointSize,
                        QString* errorMessage);
    bool loadPotreeNode(const PointCloudDataset& dataset,
                        const PointCloudNodeData& data,
                        float pointSize,
                        QString* errorMessage);
    void beginPotreeLayer(const PointCloudDataset& dataset, float pointSize);
    bool attachPotreeNode(const std::string& nodeId,
                          std::uint32_t level,
                          const BoundingBox& bounds,
                          const PointCloudDataset& dataset,
                          const PointCloudNodeData& data,
                          float pointSize,
                          QString* errorMessage);
    void setPotreeNodeVisible(const std::string& nodeId, bool visible);
    void removePotreeNode(const std::string& nodeId);
    bool hasPotreeNode(const std::string& nodeId) const;
    void clear();
    void setPointSize(float pointSize);
    void setPotreeColorMode(PotreeColorMode mode);

    QString currentFilePath() const;
    std::uint64_t pointCount() const;

private:
    void applyPointCloudState(osg::Node* node, float pointSize);
    std::uint64_t countPointsAndPrepareGeometry(osg::Node* node) const;

    osg::ref_ptr<osg::Group> m_root;
    osg::ref_ptr<osg::Node> m_pointCloudNode;
    std::unique_ptr<PotreeSceneBackend> m_potreeBackend;
    QString m_currentFilePath;
    std::uint64_t m_pointCount = 0;
};
