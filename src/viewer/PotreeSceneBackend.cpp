#include "viewer/PotreeSceneBackend.h"

#include "pointcloud/BoundingBox.h"
#include "pointcloud/PointCloudDataset.h"
#include "pointcloud/PointCloudNodeData.h"
#include "viewer/PotreeRenderMasks.h"

#include <osg/Array>
#include <osg/BoundingSphere>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/GL>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/PrimitiveSet>
#include <osg/StateAttribute>
#include <osg/StateSet>

#include <array>
#include <limits>

namespace
{
void applyPointState(osg::Node* node, float pointSize)
{
    if (!node) {
        return;
    }

    osg::StateSet* stateSet = node->getOrCreateStateSet();
    stateSet->setMode(GL_LIGHTING,
                      osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    stateSet->setAttributeAndModes(new osg::Point(pointSize),
                                   osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
}

osg::Vec4ub levelColor(std::uint32_t level)
{
    static const std::array<osg::Vec4ub, 8> palette = {
        osg::Vec4ub(230, 57, 70, 255),
        osg::Vec4ub(255, 183, 3, 255),
        osg::Vec4ub(56, 176, 0, 255),
        osg::Vec4ub(0, 180, 216, 255),
        osg::Vec4ub(58, 134, 255, 255),
        osg::Vec4ub(186, 104, 200, 255),
        osg::Vec4ub(251, 133, 0, 255),
        osg::Vec4ub(255, 255, 255, 255),
    };
    return palette[level % palette.size()];
}

osg::ref_ptr<osg::Geode> createBoundingBoxGeode(const BoundingBox& bounds,
                                                 const osg::Vec3d& origin,
                                                 std::uint32_t level)
{
    const osg::Vec3d minimum = bounds.min - origin;
    const osg::Vec3d maximum = bounds.max - origin;

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->reserve(8);
    vertices->push_back(osg::Vec3(minimum.x(), minimum.y(), minimum.z()));
    vertices->push_back(osg::Vec3(maximum.x(), minimum.y(), minimum.z()));
    vertices->push_back(osg::Vec3(maximum.x(), maximum.y(), minimum.z()));
    vertices->push_back(osg::Vec3(minimum.x(), maximum.y(), minimum.z()));
    vertices->push_back(osg::Vec3(minimum.x(), minimum.y(), maximum.z()));
    vertices->push_back(osg::Vec3(maximum.x(), minimum.y(), maximum.z()));
    vertices->push_back(osg::Vec3(maximum.x(), maximum.y(), maximum.z()));
    vertices->push_back(osg::Vec3(minimum.x(), maximum.y(), maximum.z()));

    static const std::array<unsigned char, 24> edgeIndices = {
        0, 1, 1, 2, 2, 3, 3, 0,
        4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7,
    };
    osg::ref_ptr<osg::DrawElementsUByte> edges = new osg::DrawElementsUByte(GL_LINES);
    edges->assign(edgeIndices.begin(), edgeIndices.end());

    osg::ref_ptr<osg::Vec4ubArray> colors = new osg::Vec4ubArray;
    colors->push_back(levelColor(level));

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexArray(vertices.get());
    geometry->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
    geometry->addPrimitiveSet(edges.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setNodeMask(PotreeRenderMasks::BoundingBoxes);
    geode->getOrCreateStateSet()->setAttributeAndModes(
        new osg::LineWidth(1.5f), osg::StateAttribute::ON);
    geode->addDrawable(geometry.get());
    return geode;
}
} // namespace

PotreeSceneBackend::PotreeSceneBackend(osg::Group* root)
    : m_root(root)
{
}

void PotreeSceneBackend::beginLayer(const PointCloudDataset& dataset, float pointSize)
{
    clear();
    m_layerTransform = new osg::MatrixTransform;
    m_layerTransform->setMatrix(osg::Matrixd::translate(dataset.offset));

    const osg::Vec3d localMin = dataset.bounds.min - dataset.offset;
    const osg::Vec3d localMax = dataset.bounds.max - dataset.offset;
    const osg::Vec3d localCenter = (localMin + localMax) * 0.5;
    osg::ref_ptr<osg::Node> boundsNode = new osg::Node;
    boundsNode->setInitialBound(
        osg::BoundingSphere(localCenter, (localMax - localCenter).length()));
    m_layerTransform->addChild(boundsNode.get());

    applyPointState(m_layerTransform.get(), pointSize);
    if (m_root) {
        m_root->addChild(m_layerTransform.get());
    }
}

bool PotreeSceneBackend::attachNode(const std::string& nodeId,
                                    std::uint32_t level,
                                    const BoundingBox& bounds,
                                    const PointCloudDataset& dataset,
                                    const PointCloudNodeData& data,
                                    float pointSize,
                                    QString* errorMessage)
{
    if (!m_layerTransform.valid()) {
        beginLayer(dataset, pointSize);
    }

    if (data.positions.empty() || data.positions.size() != data.colors.size()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Decoded Potree node has invalid position/color arrays.");
        }
        return false;
    }

    if (data.positions.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Decoded Potree node has too many points for one OSG draw call.");
        }
        return false;
    }

    if (hasNode(nodeId)) {
        setNodeVisible(nodeId, true);
        return true;
    }

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->assign(data.positions.begin(), data.positions.end());

    osg::ref_ptr<osg::Vec4ubArray> originalColors = new osg::Vec4ubArray;
    originalColors->assign(data.colors.begin(), data.colors.end());

    osg::ref_ptr<osg::Vec4ubArray> levelColors = new osg::Vec4ubArray;
    levelColors->assign(data.colors.size(), levelColor(level));

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexArray(vertices.get());
    geometry->setColorArray(
        m_colorMode == PotreeColorMode::LodLevel ? levelColors.get() : originalColors.get(),
        osg::Array::BIND_PER_VERTEX);
    geometry->addPrimitiveSet(new osg::DrawArrays(
        GL_POINTS,
        0,
        static_cast<GLsizei>(data.positions.size())));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setName(nodeId + ":points");
    geode->setNodeMask(PotreeRenderMasks::Points);
    geode->addDrawable(geometry.get());

    osg::ref_ptr<osg::Geode> boundingBoxGeode = createBoundingBoxGeode(
        bounds, data.origin, level);
    boundingBoxGeode->setName(nodeId + ":bounds");

    osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
    transform->setName(nodeId);
    transform->setNodeMask(PotreeRenderMasks::VisibleNode);
    transform->setMatrix(osg::Matrixd::translate(data.origin - dataset.offset));
    transform->addChild(geode.get());
    transform->addChild(boundingBoxGeode.get());
    applyPointState(transform.get(), pointSize);

    NodeVisual visual;
    visual.transform = transform;
    visual.geometry = geometry;
    visual.originalColors = originalColors;
    visual.levelColors = levelColors;
    visual.pointCount = static_cast<std::uint64_t>(data.positions.size());
    visual.gpuBytes = data.cpuBytes();
    m_layerTransform->addChild(transform.get());
    m_nodes.emplace(nodeId, std::move(visual));
    return true;
}

void PotreeSceneBackend::setNodeVisible(const std::string& nodeId, bool visible)
{
    const auto it = m_nodes.find(nodeId);
    if (it == m_nodes.end()) {
        return;
    }

    it->second.transform->setNodeMask(
        visible ? PotreeRenderMasks::VisibleNode : 0u);
}

void PotreeSceneBackend::removeNode(const std::string& nodeId)
{
    const auto it = m_nodes.find(nodeId);
    if (it == m_nodes.end()) {
        return;
    }

    if (m_layerTransform.valid()) {
        m_layerTransform->removeChild(it->second.transform.get());
    }
    m_nodes.erase(it);
}

void PotreeSceneBackend::clear()
{
    if (m_root && m_layerTransform.valid()) {
        m_root->removeChild(m_layerTransform.get());
    }
    m_nodes.clear();
    m_layerTransform = nullptr;
}

void PotreeSceneBackend::setPointSize(float pointSize)
{
    applyPointState(m_layerTransform.get(), pointSize);
    for (auto& entry : m_nodes) {
        applyPointState(entry.second.transform.get(), pointSize);
    }
}

void PotreeSceneBackend::setColorMode(PotreeColorMode mode)
{
    if (m_colorMode == mode) {
        return;
    }

    m_colorMode = mode;
    for (auto& entry : m_nodes) {
        applyColorMode(entry.second);
    }
}

void PotreeSceneBackend::applyColorMode(NodeVisual& visual)
{
    if (!visual.geometry.valid()) {
        return;
    }

    osg::Vec4ubArray* colors = m_colorMode == PotreeColorMode::LodLevel
        ? visual.levelColors.get()
        : visual.originalColors.get();
    visual.geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    visual.geometry->dirtyGLObjects();
}

bool PotreeSceneBackend::hasNode(const std::string& nodeId) const
{
    return m_nodes.find(nodeId) != m_nodes.end();
}

std::uint64_t PotreeSceneBackend::pointCount() const
{
    std::uint64_t total = 0;
    for (const auto& entry : m_nodes) {
        total += entry.second.pointCount;
    }
    return total;
}

std::uint64_t PotreeSceneBackend::gpuBytes() const
{
    std::uint64_t total = 0;
    for (const auto& entry : m_nodes) {
        total += entry.second.gpuBytes;
    }
    return total;
}

std::size_t PotreeSceneBackend::residentNodeCount() const
{
    return m_nodes.size();
}
