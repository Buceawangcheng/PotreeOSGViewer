#pragma once

#include "viewer/PotreeColorMode.h"

#include <osg/Array>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/ref_ptr>

#include <QString>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

class PointCloudDataset;
class PointCloudShaderState;
struct BoundingBox;
struct PointCloudNodeData;

// Potree 节点到 OSG 场景图的主线程后端。
//
// attachNode() 把解码后的 CPU 数组构造成 Geometry/VBO 状态并加入 layer；
// setNodeVisible() 通过 NodeMask 控制本帧是否参与 cull/draw；removeNode() 从场景树
// 和 resident 映射中删除节点。所有接口都由 PointCloudRuntime/SceneManager 在
// SingleThreaded update traversal 中调用，不能从 NodeLoadScheduler 工作线程调用。
class PotreeSceneBackend {
public:
    explicit PotreeSceneBackend(osg::Group* root);
    ~PotreeSceneBackend();

    bool initializeShader(QString* errorMessage);

    void beginLayer(const PointCloudDataset& dataset, float pointSize);
    bool attachNode(const std::string& nodeId,
                    std::uint32_t level,
                    const BoundingBox& bounds,
                    const PointCloudDataset& dataset,
                    const PointCloudNodeData& data,
                    float pointSize,
                    QString* errorMessage);
    void setNodeVisible(const std::string& nodeId, bool visible);
    void removeNode(const std::string& nodeId);
    void clear();
    void setPointSize(float pointSize);
    void setColorMode(PotreeColorMode mode);

    bool hasNode(const std::string& nodeId) const;
    std::uint64_t pointCount() const;
    std::uint64_t gpuBytes() const;
    std::size_t residentNodeCount() const;

private:
    // 一个 resident 节点对应的 OSG 资源及缓存统计。
    struct NodeVisual {
        osg::ref_ptr<osg::MatrixTransform> transform;
        osg::ref_ptr<osg::Geometry> geometry;
        osg::ref_ptr<osg::Vec4ubArray> originalColors;
        osg::ref_ptr<osg::Vec4ubArray> levelColors;
        std::uint64_t pointCount = 0;
        std::uint64_t gpuBytes = 0;
    };

    void applyColorMode(NodeVisual& visual);

    osg::Group* m_root = nullptr;
    osg::ref_ptr<osg::MatrixTransform> m_layerTransform;
    std::unique_ptr<PointCloudShaderState> m_shaderState;
    std::unordered_map<std::string, NodeVisual> m_nodes;
    PotreeColorMode m_colorMode = PotreeColorMode::OriginalRgb;
    float m_pointSize = 3.0f;
    float m_heightMinimum = 0.0f;
    float m_heightMaximum = 1.0f;
};
