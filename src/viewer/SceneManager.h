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

// 应用层场景门面。PointCloudRuntime 通过它调用 PotreeSceneBackend，而不直接依赖
// Geometry/VBO 的具体构造方式；这些接口仍属于主/update线程边界。
class SceneManager
{
public:
    SceneManager();
    ~SceneManager();

    osg::Group* root() const;
    bool initializePotreeShader(QString* errorMessage);

    bool loadPointCloud(const std::string& nativeFilePath,
                        const QString& displayFilePath,
                        float pointSize,
                        QString* errorMessage);
    bool loadPotreeNode(const PointCloudDataset& dataset,
                        const PointCloudNodeData& data,
                        float pointSize,
                        QString* errorMessage);
    // 开始新的流式 Potree layer；后续节点由 attachPotreeNode() 增量加入。
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
