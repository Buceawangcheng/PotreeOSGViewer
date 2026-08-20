#include "viewer/SceneManager.h"

#include "pointcloud/PointCloudDataset.h"
#include "pointcloud/PointCloudNodeData.h"
#include "viewer/PotreeSceneBackend.h"

#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/GL>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/Point>
#include <osg/PrimitiveSet>
#include <osg/StateAttribute>
#include <osg/StateSet>
#include <osgDB/ReadFile>

#include <limits>
#include <memory>

namespace
{
class GeometryPrepareVisitor : public osg::NodeVisitor
{
public:
    GeometryPrepareVisitor()
        : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
    }

    void apply(osg::Geode& geode) override
    {
        for (unsigned int i = 0; i < geode.getNumDrawables(); ++i) {
            osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
            if (!geometry) {
                continue;
            }

            geometry->setUseDisplayList(false);
            geometry->setUseVertexBufferObjects(true);

            const osg::Array* vertices = geometry->getVertexArray();
            if (vertices) {
                pointCount += vertices->getNumElements();
            }
        }

        traverse(geode);
    }

    std::uint64_t pointCount = 0;
};
} // namespace

SceneManager::SceneManager()
    : m_root(new osg::Group)
    , m_potreeBackend(std::make_unique<PotreeSceneBackend>(m_root.get()))
{
}

SceneManager::~SceneManager() = default;

osg::Group* SceneManager::root() const
{
    return m_root.get();
}

bool SceneManager::loadPointCloud(const std::string& nativeFilePath,
                                  const QString& displayFilePath,
                                  float pointSize,
                                  QString* errorMessage)
{
    osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(nativeFilePath);
    if (!node.valid()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to load point cloud:\n%1").arg(displayFilePath);
        }
        return false;
    }

    applyPointCloudState(node.get(), pointSize);
    const std::uint64_t pointCount = countPointsAndPrepareGeometry(node.get());

    clear();
    m_root->addChild(node.get());
    m_pointCloudNode = node;
    m_currentFilePath = displayFilePath;
    m_pointCount = pointCount;
    return true;
}

bool SceneManager::loadPotreeNode(const PointCloudDataset& dataset,
                                  const PointCloudNodeData& data,
                                  float pointSize,
                                  QString* errorMessage)
{
    clear();
    m_potreeBackend->beginLayer(dataset, pointSize);
    if (!m_potreeBackend->attachNode(
            "r", 0, dataset.bounds, dataset, data, pointSize, errorMessage)) {
        clear();
        return false;
    }
    m_currentFilePath = dataset.sourcePath;
    m_pointCount = m_potreeBackend->pointCount();
    return true;
}

void SceneManager::beginPotreeLayer(const PointCloudDataset& dataset, float pointSize)
{
    clear();
    m_potreeBackend->beginLayer(dataset, pointSize);
    m_currentFilePath = dataset.sourcePath;
    m_pointCount = 0;
}

bool SceneManager::attachPotreeNode(const std::string& nodeId,
                                    std::uint32_t level,
                                    const BoundingBox& bounds,
                                    const PointCloudDataset& dataset,
                                    const PointCloudNodeData& data,
                                    float pointSize,
                                    QString* errorMessage)
{
    if (!m_potreeBackend->attachNode(
            nodeId, level, bounds, dataset, data, pointSize, errorMessage)) {
        return false;
    }

    m_currentFilePath = dataset.sourcePath;
    m_pointCount = m_potreeBackend->pointCount();
    return true;
}

void SceneManager::setPotreeNodeVisible(const std::string& nodeId, bool visible)
{
    m_potreeBackend->setNodeVisible(nodeId, visible);
}

void SceneManager::removePotreeNode(const std::string& nodeId)
{
    m_potreeBackend->removeNode(nodeId);
    m_pointCount = m_potreeBackend->pointCount();
}

bool SceneManager::hasPotreeNode(const std::string& nodeId) const
{
    return m_potreeBackend->hasNode(nodeId);
}

void SceneManager::clear()
{
    if (m_pointCloudNode.valid()) {
        m_root->removeChild(m_pointCloudNode.get());
    }
    if (m_potreeBackend) {
        m_potreeBackend->clear();
    }

    m_pointCloudNode = nullptr;
    m_currentFilePath.clear();
    m_pointCount = 0;
}

void SceneManager::setPointSize(float pointSize)
{
    if (m_pointCloudNode.valid()) {
        applyPointCloudState(m_pointCloudNode.get(), pointSize);
    }
    if (m_potreeBackend) {
        m_potreeBackend->setPointSize(pointSize);
    }
}

void SceneManager::setPotreeColorMode(PotreeColorMode mode)
{
    if (m_potreeBackend) {
        m_potreeBackend->setColorMode(mode);
    }
}

QString SceneManager::currentFilePath() const
{
    return m_currentFilePath;
}

std::uint64_t SceneManager::pointCount() const
{
    return m_pointCount;
}

void SceneManager::applyPointCloudState(osg::Node* node, float pointSize)
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

std::uint64_t SceneManager::countPointsAndPrepareGeometry(osg::Node* node) const
{
    if (!node) {
        return 0;
    }

    GeometryPrepareVisitor visitor;
    node->accept(visitor);
    return visitor.pointCount;
}
