#include "viewer/SceneManager.h"

#include "pointcloud/PointCloudDataset.h"
#include "pointcloud/PointCloudNodeData.h"

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
{
}

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

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->assign(data.positions.begin(), data.positions.end());

    osg::ref_ptr<osg::Vec4ubArray> colors = new osg::Vec4ubArray;
    colors->assign(data.colors.begin(), data.colors.end());

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexArray(vertices.get());
    geometry->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);
    geometry->addPrimitiveSet(new osg::DrawArrays(
        GL_POINTS,
        0,
        static_cast<GLsizei>(data.positions.size())));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geometry.get());

    osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
    transform->setMatrix(osg::Matrixd::translate(data.origin));
    transform->addChild(geode.get());
    applyPointCloudState(transform.get(), pointSize);

    clear();
    m_root->addChild(transform.get());
    m_pointCloudNode = transform;
    m_currentFilePath = dataset.sourcePath;
    m_pointCount = static_cast<std::uint64_t>(data.positions.size());
    return true;
}

void SceneManager::clear()
{
    if (m_pointCloudNode.valid()) {
        m_root->removeChild(m_pointCloudNode.get());
    }

    m_pointCloudNode = nullptr;
    m_currentFilePath.clear();
    m_pointCount = 0;
}

void SceneManager::setPointSize(float pointSize)
{
    if (!m_pointCloudNode.valid()) {
        return;
    }

    applyPointCloudState(m_pointCloudNode.get(), pointSize);
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
