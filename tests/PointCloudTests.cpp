#include "pointcloud/Potree2Provider.h"
#include "pointcloud/LodSelector.h"
#include "pointcloud/NodeLoadScheduler.h"
#include "viewer/PotreeRenderMasks.h"
#include "viewer/SceneManager.h"
#include "viewer/camera/CameraMath.h"
#include "viewer/camera/CesiumCameraManipulator.h"
#include "viewer/camera/DepthBufferPicker.h"
#include "viewer/camera/PickDebugVisualizer.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QThread>
#include <QtEndian>

#include <osg/Camera>
#include <osg/AutoTransform>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Math>
#include <osg/Program>
#include <osg/View>
#include <osg/Switch>
#include <osg/observer_ptr>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/TrackballManipulator>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

namespace
{
int failures = 0;

void expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool near(float actual, float expected)
{
    return std::abs(actual - expected) < 0.0001f;
}

bool nearDouble(double actual, double expected)
{
    return std::abs(actual - expected) < 0.0001;
}

bool nearMatrix(const osg::Matrixd& actual,
                const osg::Matrixd& expected,
                double epsilon = 1.0e-9)
{
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            if (std::abs(actual(row, column) - expected(row, column)) > epsilon) {
                return false;
            }
        }
    }
    return true;
}

class TestGuiActionAdapter : public osgGA::GUIActionAdapter
{
public:
    osg::View* asView() override
    {
        return view;
    }

    void requestRedraw() override
    {
        redrawRequested = true;
    }

    void requestContinuousUpdate(bool needed = true) override
    {
        continuousUpdateRequested = needed;
    }

    void requestWarpPointer(float, float) override
    {
    }

    bool redrawRequested = false;
    bool continuousUpdateRequested = true;
    osg::View* view = nullptr;
};

class TestDepthBufferPicker : public DepthBufferPicker
{
public:
    bool takeRequest(DepthPickRequest& request)
    {
        return takePendingRequest(request);
    }

    void emitResult(const DepthPickResult& result)
    {
        publishResult(result);
    }

protected:
    ~TestDepthBufferPicker() override = default;
};

bool writeFile(const QString& path, const QByteArray& bytes)
{
    QFile file(path);
    return file.open(QIODevice::WriteOnly)
        && file.write(bytes) == bytes.size();
}

QJsonArray vec3(double x, double y, double z)
{
    return QJsonArray { x, y, z };
}

QJsonObject attribute(const QString& name,
                      int size,
                      int elements,
                      int elementSize,
                      const QString& type)
{
    QJsonObject value;
    value.insert(QStringLiteral("name"), name);
    value.insert(QStringLiteral("description"), QString());
    value.insert(QStringLiteral("size"), size);
    value.insert(QStringLiteral("numElements"), elements);
    value.insert(QStringLiteral("elementSize"), elementSize);
    value.insert(QStringLiteral("type"), type);
    value.insert(QStringLiteral("min"), QJsonArray { 0 });
    value.insert(QStringLiteral("max"), QJsonArray { 65535 });
    value.insert(QStringLiteral("scale"), QJsonArray { 1 });
    value.insert(QStringLiteral("offset"), QJsonArray { 0 });
    return value;
}

QByteArray metadataBytes(int firstChunkSize, const QString& encoding)
{
    QJsonObject hierarchy;
    hierarchy.insert(QStringLiteral("firstChunkSize"), firstChunkSize);
    hierarchy.insert(QStringLiteral("stepSize"), 4);
    hierarchy.insert(QStringLiteral("depth"), 1);

    QJsonObject bounds;
    bounds.insert(QStringLiteral("min"), vec3(100.0, 200.0, 300.0));
    bounds.insert(QStringLiteral("max"), vec3(101.0, 201.0, 301.0));

    QJsonArray attributes;
    QJsonObject position = attribute(QStringLiteral("position"), 12, 3, 4, QStringLiteral("int32"));
    position.insert(QStringLiteral("min"), vec3(100.0, 200.0, 300.0));
    position.insert(QStringLiteral("max"), vec3(101.0, 201.0, 301.0));
    attributes.append(position);
    attributes.append(attribute(QStringLiteral("rgb"), 6, 3, 2, QStringLiteral("uint16")));

    QJsonObject root;
    root.insert(QStringLiteral("version"), QStringLiteral("2.0"));
    root.insert(QStringLiteral("name"), QStringLiteral("fixture"));
    root.insert(QStringLiteral("points"), 2);
    root.insert(QStringLiteral("encoding"), encoding);
    root.insert(QStringLiteral("spacing"), 1.0);
    root.insert(QStringLiteral("offset"), vec3(100.0, 200.0, 300.0));
    root.insert(QStringLiteral("scale"), vec3(0.01, 0.01, 0.01));
    root.insert(QStringLiteral("hierarchy"), hierarchy);
    root.insert(QStringLiteral("boundingBox"), bounds);
    root.insert(QStringLiteral("attributes"), attributes);
    return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

QByteArray hierarchyRecord(std::uint8_t type,
                           std::uint32_t points,
                           std::uint64_t offset,
                           std::uint64_t size,
                           std::uint8_t childMask = 0)
{
    QByteArray bytes(22, '\0');
    uchar* output = reinterpret_cast<uchar*>(bytes.data());
    output[0] = type;
    output[1] = childMask;
    qToLittleEndian<std::uint32_t>(points, output + 2);
    qToLittleEndian<std::uint64_t>(offset, output + 6);
    qToLittleEndian<std::uint64_t>(size, output + 14);
    return bytes;
}

void appendPoint(QByteArray* bytes,
                 std::int32_t x,
                 std::int32_t y,
                 std::int32_t z,
                 std::uint16_t r,
                 std::uint16_t g,
                 std::uint16_t b)
{
    const int start = bytes->size();
    bytes->resize(start + 18);
    uchar* output = reinterpret_cast<uchar*>(bytes->data() + start);
    qToLittleEndian<std::uint32_t>(static_cast<std::uint32_t>(x), output);
    qToLittleEndian<std::uint32_t>(static_cast<std::uint32_t>(y), output + 4);
    qToLittleEndian<std::uint32_t>(static_cast<std::uint32_t>(z), output + 8);
    qToLittleEndian<std::uint16_t>(r, output + 12);
    qToLittleEndian<std::uint16_t>(g, output + 14);
    qToLittleEndian<std::uint16_t>(b, output + 16);
}

bool createFixture(const QString& directory,
                   int firstChunkSize = 22,
                   const QString& encoding = QStringLiteral("DEFAULT"),
                   std::uint64_t octreeOffset = 0)
{
    QByteArray points;
    appendPoint(&points, 0, 0, 0, 255, 256, 65535);
    appendPoint(&points, 100, 50, 25, 0, 128, 32768);

    return writeFile(QDir(directory).filePath(QStringLiteral("metadata.json")),
                     metadataBytes(firstChunkSize, encoding))
        && writeFile(QDir(directory).filePath(QStringLiteral("hierarchy.bin")),
                     firstChunkSize == 22
                         ? hierarchyRecord(1, 2, octreeOffset, 36)
                         : QByteArray(firstChunkSize, '\0'))
        && writeFile(QDir(directory).filePath(QStringLiteral("octree.bin")), points);
}

bool createProxyFixture(const QString& directory)
{
    QByteArray points;
    appendPoint(&points, 0, 0, 0, 255, 0, 0);
    appendPoint(&points, 50, 0, 0, 0, 255, 0);
    appendPoint(&points, 25, 25, 0, 0, 0, 255);

    QByteArray hierarchy;
    hierarchy.append(hierarchyRecord(0, 1, 0, 18, 0b00000001));
    hierarchy.append(hierarchyRecord(2, 0, 44, 44));
    hierarchy.append(hierarchyRecord(0, 1, 18, 18, 0b00000001));
    hierarchy.append(hierarchyRecord(1, 1, 36, 18));

    return writeFile(QDir(directory).filePath(QStringLiteral("metadata.json")),
                     metadataBytes(44, QStringLiteral("DEFAULT")))
        && writeFile(QDir(directory).filePath(QStringLiteral("hierarchy.bin")), hierarchy)
        && writeFile(QDir(directory).filePath(QStringLiteral("octree.bin")), points);
}

void collectProxyNodes(OctreeNode* node, std::vector<OctreeNode*>* proxies)
{
    if (!node) {
        return;
    }
    if (node->hierarchyState == HierarchyState::Proxy) {
        proxies->push_back(node);
    }
    for (std::unique_ptr<OctreeNode>& child : node->children) {
        collectProxyNodes(child.get(), proxies);
    }
}

struct HierarchyStats {
    std::uint64_t nodes = 0;
    std::uint64_t points = 0;
    std::uint32_t maxLevel = 0;
};

void collectHierarchyStats(const OctreeNode* node, HierarchyStats* stats)
{
    if (!node) {
        return;
    }
    ++stats->nodes;
    stats->points += node->pointCount;
    stats->maxLevel = std::max(stats->maxLevel, node->level);
    for (const std::unique_ptr<OctreeNode>& child : node->children) {
        collectHierarchyStats(child.get(), stats);
    }
}

BoundingBox box(double minX,
                double minY,
                double minZ,
                double maxX,
                double maxY,
                double maxZ)
{
    return BoundingBox {
        osg::Vec3d(minX, minY, minZ),
        osg::Vec3d(maxX, maxY, maxZ),
    };
}

CameraState testCamera()
{
    CameraState camera;
    camera.viewMatrix.makeLookAt(osg::Vec3d(0.0, -10.0, 0.0),
                                 osg::Vec3d(0.0, 0.0, 0.0),
                                 osg::Vec3d(0.0, 0.0, 1.0));
    camera.projectionMatrix.makePerspective(90.0, 1.0, 0.1, 1000.0);
    camera.position = osg::Vec3d(0.0, -10.0, 0.0);
    camera.viewportHeight = 100;
    return camera;
}

const osg::Vec4ubArray* potreeNodeColors(SceneManager& scene,
                                         unsigned int layerChildIndex)
{
    if (scene.root()->getNumChildren() == 0) {
        return nullptr;
    }

    osg::Group* layer = scene.root()->getChild(0)->asGroup();
    if (!layer || layerChildIndex >= layer->getNumChildren()) {
        return nullptr;
    }

    osg::Group* nodeTransform = layer->getChild(layerChildIndex)->asGroup();
    if (!nodeTransform || nodeTransform->getNumChildren() == 0) {
        return nullptr;
    }

    osg::Geode* geode = dynamic_cast<osg::Geode*>(nodeTransform->getChild(0));
    if (!geode || geode->getNumDrawables() == 0) {
        return nullptr;
    }

    osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(geode->getDrawable(0));
    return geometry
        ? dynamic_cast<const osg::Vec4ubArray*>(geometry->getColorArray())
        : nullptr;
}

osg::StateSet* potreeNodePointState(SceneManager& scene,
                                    unsigned int layerChildIndex)
{
    if (scene.root()->getNumChildren() == 0) {
        return nullptr;
    }
    osg::Group* layer = scene.root()->getChild(0)->asGroup();
    osg::Group* nodeTransform = layer && layerChildIndex < layer->getNumChildren()
        ? layer->getChild(layerChildIndex)->asGroup()
        : nullptr;
    osg::Geode* geode = nodeTransform && nodeTransform->getNumChildren() > 0
        ? dynamic_cast<osg::Geode*>(nodeTransform->getChild(0))
        : nullptr;
    return geode ? geode->getStateSet() : nullptr;
}

void testValidDefaultDataset()
{
    QTemporaryDir fixture;
    expect(fixture.isValid(), "temporary directory should be available");
    expect(createFixture(fixture.path()), "valid fixture should be written");

    Potree2Provider provider;
    QString error;
    std::shared_ptr<PointCloudDataset> dataset = provider.openMetadata(fixture.path(), &error);
    expect(dataset != nullptr, "valid directory should open");
    if (!dataset) {
        std::cerr << error.toStdString() << '\n';
        return;
    }

    expect(dataset->attributes.pointRecordSizeBytes() == 18, "record size should preserve attribute order");
    expect(dataset->root->pointCount == 2, "root point count should come from hierarchy");
    expect(dataset->root->pointDataState == PointDataState::Unloaded,
           "hierarchy parsing should not mark point data ready");
    expect(dataset->root->hierarchyState == HierarchyState::Resolved,
           "normal root hierarchy should be resolved after first chunk parsing");
    expect(dataset->root->pointByteOffset == 0 && dataset->root->pointByteSize == 36,
           "normal root should keep octree point-data range separate");

    std::shared_ptr<PointCloudNodeData> data = provider.loadNodeData(*dataset, dataset->root.get(), &error);
    expect(data != nullptr, "DEFAULT root node should decode");
    if (!data) {
        std::cerr << error.toStdString() << '\n';
        return;
    }

    expect(data->positions.size() == 2, "two positions should be decoded");
    expect(near(data->positions[0].x(), 0.0f)
               && near(data->positions[0].y(), 0.0f)
               && near(data->positions[0].z(), 0.0f),
           "first point should be local to dataset origin");
    expect(near(data->positions[1].x(), 1.0f)
               && near(data->positions[1].y(), 0.5f)
               && near(data->positions[1].z(), 0.25f),
           "scaled second position should decode");
    expect(data->colors[0].r() == 255 && data->colors[0].g() == 1 && data->colors[0].b() == 255,
           "16-bit RGB should follow Potree conversion");
    expect(data->colors[1].r() == 0 && data->colors[1].g() == 128 && data->colors[1].b() == 128,
           "8-bit-range and 16-bit-range RGB should both decode");
    expect(dataset->root->pointDataState == PointDataState::CpuReady,
           "decoded node should be CPU ready");

    SceneManager scene;
    expect(scene.loadPotreeNode(*dataset, *data, 3.0f, &error),
           "decoded Potree node should create OSG geometry");
    expect(scene.pointCount() == 2 && scene.root()->getNumChildren() == 1,
           "Potree OSG scene should expose the decoded point count");
    scene.clear();
    expect(scene.pointCount() == 0 && scene.root()->getNumChildren() == 0,
           "Potree OSG scene should clear cleanly");

    const QString metadataPath = QDir(fixture.path()).filePath(QStringLiteral("metadata.json"));
    expect(provider.openMetadata(metadataPath, &error) != nullptr,
           "metadata.json file path should also open");
}

void testPlySceneRegression()
{
    QTemporaryDir fixture;
    const QByteArray ply(
        "ply\n"
        "format ascii 1.0\n"
        "element vertex 2\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
        "0 0 0 255 0 0\n"
        "1 2 3 0 255 0\n");
    const QString path = QDir(fixture.path()).filePath(QStringLiteral("fixture.ply"));
    expect(writeFile(path, ply), "PLY fixture should be written");

    SceneManager scene;
    QString error;
    const QByteArray nativePath = QFile::encodeName(path);
    expect(scene.loadPointCloud(nativePath.constData(), path, 3.0f, &error),
           "existing PLY scene path should still load");
    if (scene.pointCount() != 2) {
        std::cerr << error.toStdString() << '\n';
    }
    expect(scene.pointCount() == 2 && scene.root()->getNumChildren() == 1,
           "PLY scene should report two points");
    scene.setPointSize(5.0f);
    scene.clear();
    expect(scene.pointCount() == 0 && scene.root()->getNumChildren() == 0,
           "PLY scene should clear cleanly");
}

void testMissingHierarchy()
{
    QTemporaryDir fixture;
    writeFile(QDir(fixture.path()).filePath(QStringLiteral("metadata.json")),
              metadataBytes(22, QStringLiteral("DEFAULT")));
    writeFile(QDir(fixture.path()).filePath(QStringLiteral("octree.bin")), QByteArray(36, '\0'));

    Potree2Provider provider;
    QString error;
    expect(provider.openMetadata(fixture.path(), &error) == nullptr,
           "missing hierarchy should fail");
    expect(error.contains(QStringLiteral("Missing hierarchy.bin")),
           "missing hierarchy error should be clear");
}

void testInvalidHierarchyChunkSize()
{
    QTemporaryDir fixture;
    expect(createFixture(fixture.path(), 21), "invalid hierarchy fixture should be written");

    Potree2Provider provider;
    QString error;
    expect(provider.openMetadata(fixture.path(), &error) == nullptr,
           "non-record-aligned hierarchy chunk should fail");
    expect(error.contains(QStringLiteral("not divisible")),
           "invalid hierarchy size error should be clear");
}

void testOctreeRangeAndEncodingErrors()
{
    QTemporaryDir rangeFixture;
    expect(createFixture(rangeFixture.path(), 22, QStringLiteral("DEFAULT"), 100),
           "out-of-range fixture should be written");

    Potree2Provider provider;
    QString error;
    std::shared_ptr<PointCloudDataset> rangeDataset = provider.openMetadata(rangeFixture.path(), &error);
    expect(rangeDataset != nullptr, "out-of-range data should not invalidate metadata");
    if (rangeDataset) {
        expect(provider.loadNodeData(*rangeDataset, rangeDataset->root.get(), &error) == nullptr,
               "out-of-file node range should fail decoding");
        expect(error.contains(QStringLiteral("lies outside")),
               "out-of-file range error should be clear");
    }

    QTemporaryDir brotliFixture;
    expect(createFixture(brotliFixture.path(), 22, QStringLiteral("BROTLI")),
           "BROTLI fixture should be written");
    std::shared_ptr<PointCloudDataset> brotliDataset = provider.openMetadata(brotliFixture.path(), &error);
    expect(brotliDataset != nullptr, "BROTLI metadata should remain supported");
    if (brotliDataset) {
        expect(provider.loadNodeData(*brotliDataset, brotliDataset->root.get(), &error) == nullptr,
               "BROTLI point decoding should remain unsupported");
        expect(error.contains(QStringLiteral("not supported yet")),
               "BROTLI decode boundary should be explicit");
    }
}

void testProxyHierarchyPatch()
{
    QTemporaryDir fixture;
    expect(fixture.isValid(), "temporary directory should be available");
    expect(createProxyFixture(fixture.path()), "proxy fixture should be written");

    Potree2Provider provider;
    QString error;
    std::shared_ptr<PointCloudDataset> dataset = provider.openMetadata(fixture.path(), &error);
    expect(dataset != nullptr, "proxy fixture metadata should open");
    if (!dataset) {
        std::cerr << error.toStdString() << '\n';
        return;
    }

    OctreeNode* proxy = dataset->root->children[0].get();
    expect(proxy != nullptr, "first chunk should create proxy child");
    expect(proxy->hierarchyState == HierarchyState::Proxy,
           "child should initially be an unresolved proxy");
    expect(proxy->hierarchyByteOffset == 44 && proxy->hierarchyByteSize == 44,
           "proxy should keep hierarchy range");
    expect(proxy->pointByteSize == 0,
           "proxy should not reuse hierarchy range as point range");

    HierarchyPatch patch;
    expect(provider.loadHierarchyPatch(*dataset, *proxy, &patch, &error),
           "proxy hierarchy patch should load");
    expect(patch.nodes.size() == 2 && patch.nodes[0].id == "r0" && patch.nodes[1].id == "r00",
           "proxy patch should contain BFS replacement and child");
    expect(provider.applyHierarchyPatch(dataset.get(), patch, &error),
           "proxy hierarchy patch should apply");

    proxy = dataset->root->children[0].get();
    expect(proxy->hierarchyState == HierarchyState::Resolved && proxy->type == OctreeNodeType::Normal,
           "proxy should be replaced by a resolved normal node");
    expect(proxy->pointByteOffset == 18 && proxy->pointByteSize == 18,
           "resolved proxy root should now carry point-data range");
    expect(proxy->children[0] != nullptr && proxy->children[0]->type == OctreeNodeType::Leaf,
           "proxy patch should create child nodes");
}

void testLocalSampleFullHierarchyStats()
{
    const QString dataPath = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../../data"));
    if (!QFile::exists(QDir(dataPath).filePath(QStringLiteral("metadata.json")))) {
        std::cerr << "Skipping local sample hierarchy test; data/metadata.json was not found.\n";
        return;
    }

    Potree2Provider provider;
    QString error;
    std::shared_ptr<PointCloudDataset> dataset = provider.openMetadata(dataPath, &error);
    expect(dataset != nullptr, "local sample metadata should open");
    if (!dataset) {
        std::cerr << error.toStdString() << '\n';
        return;
    }

    std::uint64_t chunks = 1;
    std::set<std::pair<std::uint64_t, std::uint64_t>> loadedChunks;
    loadedChunks.insert({0, dataset->root->hierarchyByteSize});
    for (;;) {
        std::vector<OctreeNode*> proxies;
        collectProxyNodes(dataset->root.get(), &proxies);
        if (proxies.empty()) {
            break;
        }

        OctreeNode* proxy = nullptr;
        for (OctreeNode* candidate : proxies) {
            const auto key = std::make_pair(candidate->hierarchyByteOffset,
                                            candidate->hierarchyByteSize);
            if (!loadedChunks.count(key)) {
                proxy = candidate;
                loadedChunks.insert(key);
                break;
            }
        }

        if (!proxy) {
            expect(false, "local sample should not leave duplicate unresolved proxy chunks");
            break;
        }

        HierarchyPatch patch;
        expect(provider.loadHierarchyPatch(*dataset, *proxy, &patch, &error),
               "local sample proxy chunk should load");
        if (patch.nodes.empty()) {
            std::cerr << error.toStdString() << '\n';
            return;
        }
        expect(provider.applyHierarchyPatch(dataset.get(), patch, &error),
               "local sample proxy chunk should apply");
        ++chunks;
    }

    HierarchyStats stats;
    collectHierarchyStats(dataset->root.get(), &stats);
    expect(chunks == 55, "local sample should contain 55 hierarchy chunks");
    expect(stats.nodes == 522, "local sample should contain 522 logical nodes");
    expect(stats.maxLevel == 6, "local sample max level should be 6");
    expect(stats.points == dataset->totalPoints && stats.points == 1176615,
           "local sample logical point sum should match metadata");
}

void testLodProjectionAndSelection()
{
    CameraState camera = testCamera();

    OctreeNode node;
    node.id = "r";
    node.bounds = box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    const double expectedRadius = std::sqrt(3.0) * 100.0 / 20.0;
    expect(nearDouble(LodSelector::projectedPixelRadius(node, camera), expectedRadius),
           "projected pixel radius should match perspective formula");

    camera.position = osg::Vec3d(0.0, 0.0, 0.0);
    expect(std::isinf(LodSelector::projectedPixelRadius(node, camera)),
           "camera inside node bounds should produce infinite weight");
}

void testLodFrustumAndRootFallback()
{
    OctreeNode root;
    root.id = "r";
    root.level = 0;
    root.bounds = box(-25.0, -25.0, -2.0, 25.0, 2.0, 2.0);
    root.pointCount = 1;

    auto visible = std::make_unique<OctreeNode>();
    visible->id = "r0";
    visible->level = 1;
    visible->bounds = box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5);
    visible->pointCount = 1;

    auto outside = std::make_unique<OctreeNode>();
    outside->id = "r1";
    outside->level = 1;
    outside->bounds = box(19.5, -0.5, -0.5, 20.5, 0.5, 0.5);
    outside->pointCount = 1;

    auto behind = std::make_unique<OctreeNode>();
    behind->id = "r2";
    behind->level = 1;
    behind->bounds = box(-0.5, -20.5, -0.5, 0.5, -19.5, 0.5);
    behind->pointCount = 1;

    root.children[0] = std::move(visible);
    root.children[1] = std::move(outside);
    root.children[2] = std::move(behind);

    LodSelectionSettings settings;
    settings.pointBudget = 10;
    settings.minimumNodePixelSize = 1.0;
    settings.maxLevel = 4;

    SelectionResult result = LodSelector().select(root, testCamera(), settings);
    expect(result.selectedNodes.size() == 2
               && result.selectedNodes[0].nodeId == "r"
               && result.selectedNodes[1].nodeId == "r0",
           "frustum selection should keep the visible child and reject side/behind children");

    OctreeNode fallbackRoot;
    fallbackRoot.id = "r";
    fallbackRoot.bounds = box(100.0, 100.0, 100.0, 101.0, 101.0, 101.0);
    fallbackRoot.pointCount = 1;
    result = LodSelector().select(fallbackRoot, testCamera(), settings);
    expect(result.selectedNodes.size() == 1 && result.selectedNodes[0].nodeId == "r",
           "root should remain selectable as the bootstrap fallback outside the frustum");

    const osg::Vec3d offset(508214.0, 3534647.0, 31.0);
    root.children[0]->bounds = box(offset.x() - 0.5,
                                   offset.y() - 0.5,
                                   offset.z() - 0.5,
                                   offset.x() + 0.5,
                                   offset.y() + 0.5,
                                   offset.z() + 0.5);
    root.children[1].reset();
    root.children[2].reset();

    CameraState translatedCamera;
    translatedCamera.viewMatrix.makeLookAt(offset + osg::Vec3d(0.0, -10.0, 0.0),
                                            offset,
                                            osg::Vec3d(0.0, 0.0, 1.0));
    translatedCamera.projectionMatrix.makePerspective(90.0, 1.0, 0.1, 1000.0);
    translatedCamera.position = offset + osg::Vec3d(0.0, -10.0, 0.0);
    translatedCamera.viewportHeight = 100;

    result = LodSelector().select(root, translatedCamera, settings);
    expect(result.selectedNodes.size() == 2 && result.selectedNodes[1].nodeId == "r0",
           "frustum selection should work for translated real-world coordinates");
}

void testLodBudgetAndRequests()
{
    OctreeNode root;
    root.id = "r";
    root.level = 0;
    root.bounds = box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    root.pointCount = 10;
    root.pointDataState = PointDataState::CpuReady;
    root.gpuState = GpuState::Resident;

    auto expensive = std::make_unique<OctreeNode>();
    expensive->id = "r0";
    expensive->level = 1;
    expensive->bounds = box(-0.9, -0.9, -0.9, 0.0, 0.0, 0.0);
    expensive->pointCount = 100;
    expensive->pointDataState = PointDataState::Unloaded;

    auto cheap = std::make_unique<OctreeNode>();
    cheap->id = "r1";
    cheap->level = 1;
    cheap->bounds = box(0.0, -0.9, -0.9, 0.9, 0.0, 0.0);
    cheap->pointCount = 5;
    cheap->pointDataState = PointDataState::Unloaded;

    root.children[0] = std::move(expensive);
    root.children[1] = std::move(cheap);

    LodSelectionSettings settings;
    settings.pointBudget = 20;
    settings.minimumNodePixelSize = 1.0;
    settings.maxLevel = 4;

    const SelectionResult result = LodSelector().select(root, testCamera(), settings);
    expect(result.selectedNodes.size() == 2,
           "budget skip should continue and allow later smaller nodes");
    expect(result.selectedPointCount == 15,
           "selected point budget should include selected unloaded nodes");
    expect(result.selectedNodes[0].nodeId == "r" && result.selectedNodes[1].nodeId == "r1",
           "selection should include root and affordable child");
    expect(result.loadCandidates.size() == 1 && result.loadCandidates[0].nodeId == "r1",
           "selected unloaded child should become a load candidate");

    settings.pointBudget = 5;
    const SelectionResult softRoot = LodSelector().select(root, testCamera(), settings);
    expect(softRoot.selectedNodes.size() == 1 && softRoot.selectedNodes[0].nodeId == "r",
           "root should remain selected when it exceeds point budget");
    expect(softRoot.overBudget, "root soft budget overflow should be reported");
}

void testLodPixelThresholdAndProxy()
{
    OctreeNode root;
    root.id = "r";
    root.level = 0;
    root.bounds = box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    root.pointCount = 1;

    auto child = std::make_unique<OctreeNode>();
    child->id = "r0";
    child->level = 1;
    child->bounds = box(-0.01, -0.01, -0.01, 0.01, 0.01, 0.01);
    child->pointCount = 1;
    root.children[0] = std::move(child);

    LodSelectionSettings settings;
    settings.pointBudget = 10;
    settings.minimumNodePixelSize = 1000.0;
    settings.maxLevel = 4;

    SelectionResult result = LodSelector().select(root, testCamera(), settings);
    expect(result.selectedNodes.size() == 1,
           "minimum pixel size should stop small child expansion");

    root.hierarchyState = HierarchyState::Proxy;
    root.type = OctreeNodeType::Proxy;
    root.hierarchyByteOffset = 22;
    root.hierarchyByteSize = 22;
    settings.minimumNodePixelSize = 1.0;
    result = LodSelector().select(root, testCamera(), settings);
    expect(result.selectedNodes.size() == 1,
           "proxy node should not expand children before hierarchy is resolved");
    expect(result.loadCandidates.size() == 1 && result.loadCandidates[0].hierarchyProxy,
           "selected proxy should request hierarchy loading");
}

void testNodeLoadSchedulerCompletionQueue()
{
    QTemporaryDir fixture;
    expect(fixture.isValid(), "temporary directory should be available");
    expect(createFixture(fixture.path()), "scheduler fixture should be written");

    Potree2Provider provider;
    QString error;
    std::shared_ptr<PointCloudDataset> dataset = provider.openMetadata(fixture.path(), &error);
    expect(dataset != nullptr, "scheduler fixture metadata should open");
    if (!dataset) {
        std::cerr << error.toStdString() << '\n';
        return;
    }

    NodeLoadRequest request;
    request.datasetGeneration = 7;
    request.nodeId = dataset->root->id;
    request.requestGeneration = ++dataset->root->requestGeneration;
    request.level = dataset->root->level;
    request.bounds = dataset->root->bounds;
    request.pointCount = dataset->root->pointCount;
    request.type = dataset->root->type;
    request.hierarchyState = dataset->root->hierarchyState;
    request.pointByteOffset = dataset->root->pointByteOffset;
    request.pointByteSize = dataset->root->pointByteSize;
    request.requestWeight = 10.0;

    NodeLoadScheduler scheduler;
    scheduler.setDataset(dataset);
    scheduler.setMaxConcurrentLoads(1);
    dataset->root->pointDataState = PointDataState::Queued;
    scheduler.schedule(request);

    std::vector<NodeLoadResult> completed;
    for (int attempt = 0; attempt < 100 && completed.empty(); ++attempt) {
        completed = scheduler.drainCompleted();
        if (completed.empty()) {
            QThread::msleep(10);
        }
    }

    expect(completed.size() == 1, "scheduler should publish one completed result");
    if (completed.empty()) {
        return;
    }
    expect(completed[0].datasetGeneration == 7
               && completed[0].nodeId == "r"
               && completed[0].requestGeneration == request.requestGeneration,
           "completed result should preserve validation identifiers");
    expect(completed[0].pointData && completed[0].pointData->positions.size() == 2,
           "scheduler should decode point data in worker result");
    expect(dataset->root->pointDataState == PointDataState::Queued,
           "worker should not mutate the real OctreeNode state");
}

void testPotreeSceneMultipleNodes()
{
    QTemporaryDir fixture;
    expect(fixture.isValid(), "temporary directory should be available");
    expect(createFixture(fixture.path()), "multi-node scene fixture should be written");

    Potree2Provider provider;
    QString error;
    std::shared_ptr<PointCloudDataset> dataset = provider.openMetadata(fixture.path(), &error);
    expect(dataset != nullptr, "multi-node scene metadata should open");
    if (!dataset) {
        std::cerr << error.toStdString() << '\n';
        return;
    }

    std::shared_ptr<PointCloudNodeData> data = provider.loadNodeData(*dataset, dataset->root.get(), &error);
    expect(data != nullptr, "multi-node scene data should decode");
    if (!data) {
        std::cerr << error.toStdString() << '\n';
        return;
    }

    PointCloudNodeData shifted = *data;
    shifted.origin += osg::Vec3d(1.0, 0.0, 0.0);

    SceneManager scene;
    expect(scene.initializePotreeShader(&error),
           "embedded point Shader resources should initialize");
    scene.beginPotreeLayer(*dataset, 3.0f);
    const osg::BoundingSphere initialBound = scene.root()->getBound();
    const osg::Vec3d expectedCenter = (dataset->bounds.min + dataset->bounds.max) * 0.5;
    expect(initialBound.valid()
               && (initialBound.center() - expectedCenter).length() < 1e-6,
           "empty Potree layer should expose the dataset bound for the initial camera home");
    expect(scene.attachPotreeNode(
               "r", 0, dataset->bounds, *dataset, *data, 3.0f, &error),
           "first Potree node should attach");
    BoundingBox shiftedBounds = dataset->bounds;
    shiftedBounds.min += osg::Vec3d(1.0, 0.0, 0.0);
    shiftedBounds.max += osg::Vec3d(1.0, 0.0, 0.0);
    expect(scene.attachPotreeNode(
               "r1", 1, shiftedBounds, *dataset, shifted, 3.0f, &error),
           "second Potree node should attach");
    expect(scene.pointCount() == 4 && scene.root()->getNumChildren() == 1,
           "Potree layer should contain two resident nodes under one layer");

    osg::Group* layer = scene.root()->getChild(0)->asGroup();
    osg::Group* firstNode = layer ? layer->getChild(1)->asGroup() : nullptr;
    expect(firstNode && firstNode->getNumChildren() == 2,
           "each resident node should own point and bounding-box child branches");
    expect(firstNode
               && firstNode->getChild(0)->getNodeMask() == PotreeRenderMasks::Points
               && firstNode->getChild(1)->getNodeMask() == PotreeRenderMasks::BoundingBoxes,
           "point and bounding-box children should use independent cull-mask bits");

    osg::Geode* pointGeode = firstNode && firstNode->getNumChildren() > 0
        ? dynamic_cast<osg::Geode*>(firstNode->getChild(0))
        : nullptr;
    osg::StateSet* pointStateSet = pointGeode ? pointGeode->getStateSet() : nullptr;
    const osg::Program* pointProgram = pointStateSet
        ? dynamic_cast<const osg::Program*>(
              pointStateSet->getAttribute(osg::StateAttribute::PROGRAM))
        : nullptr;
    expect(pointProgram && pointProgram->getNumShaders() == 2,
           "PointGeode should use the embedded vertex and fragment Shader program");

    osg::Geode* boundsGeode = firstNode && firstNode->getNumChildren() > 1
        ? dynamic_cast<osg::Geode*>(firstNode->getChild(1))
        : nullptr;
    osg::Geometry* boundsGeometry = boundsGeode && boundsGeode->getNumDrawables() > 0
        ? dynamic_cast<osg::Geometry*>(boundsGeode->getDrawable(0))
        : nullptr;
    const osg::StateSet* boundsStateSet = boundsGeode ? boundsGeode->getStateSet() : nullptr;
    expect(!boundsStateSet
               || !boundsStateSet->getAttribute(osg::StateAttribute::PROGRAM),
           "BoundingBoxGeode should not inherit the point Shader program");
    expect(boundsGeometry
               && boundsGeometry->getVertexArray()->getNumElements() == 8
               && boundsGeometry->getNumPrimitiveSets() == 1
               && boundsGeometry->getPrimitiveSet(0)->getMode() == GL_LINES,
           "node bounding boxes should use eight vertices and line primitives");

    const osg::Vec4ubArray* colors = potreeNodeColors(scene, 1);
    expect(colors && colors->size() == 2 && colors->at(0) != colors->at(1),
           "Potree nodes should initially use decoded RGB colors");

    scene.setPotreeColorMode(PotreeColorMode::LodLevel);
    const osg::Vec4ubArray* level0Colors = potreeNodeColors(scene, 1);
    const osg::Vec4ubArray* level1Colors = potreeNodeColors(scene, 2);
    expect(level0Colors == colors && level1Colors,
           "Shader color mode changes should keep the original Geometry color array");

    osg::StateSet* firstPointState = potreeNodePointState(scene, 1);
    osg::StateSet* secondPointState = potreeNodePointState(scene, 2);
    osg::Uniform* firstColorMode = firstPointState
        ? firstPointState->getUniform("uColorMode")
        : nullptr;
    osg::Uniform* secondColorMode = secondPointState
        ? secondPointState->getUniform("uColorMode")
        : nullptr;
    int colorMode = -1;
    expect(firstColorMode && firstColorMode == secondColorMode
               && firstColorMode->get(colorMode)
               && colorMode == static_cast<int>(PotreeColorMode::LodLevel),
           "all PointGeodes should share the LOD color mode uniform");

    int firstLevel = -1;
    int secondLevel = -1;
    osg::Uniform* firstLevelUniform = firstPointState
        ? firstPointState->getUniform("uLodLevel")
        : nullptr;
    osg::Uniform* secondLevelUniform = secondPointState
        ? secondPointState->getUniform("uLodLevel")
        : nullptr;
    expect(firstLevelUniform && secondLevelUniform
               && firstLevelUniform->get(firstLevel)
               && secondLevelUniform->get(secondLevel)
               && firstLevel == 0 && secondLevel == 1,
           "each PointGeode should retain its own LOD level uniform");

    scene.setPotreeColorMode(PotreeColorMode::Height);
    expect(firstColorMode->get(colorMode)
               && colorMode == static_cast<int>(PotreeColorMode::Height),
           "Height mode should update the shared color mode uniform");
    scene.setPointSize(7.0f);
    osg::Uniform* pointSizeUniform = firstPointState
        ? firstPointState->getUniform("uPointSize")
        : nullptr;
    float shaderPointSize = 0.0f;
    expect(pointSizeUniform && pointSizeUniform->get(shaderPointSize)
               && near(shaderPointSize, 7.0f),
           "point size should update the shared Shader uniform");

    scene.setPotreeColorMode(PotreeColorMode::OriginalRgb);
    colors = potreeNodeColors(scene, 1);
    expect(colors && colors->at(0) != colors->at(1),
           "switching back should keep decoded RGB colors attached");

    scene.setPotreeNodeVisible("r1", false);
    expect(scene.pointCount() == 4,
           "hiding a Potree node should not delete resident resources");
    osg::Group* secondNode = layer ? layer->getChild(2)->asGroup() : nullptr;
    expect(secondNode && secondNode->getNodeMask() == 0
               && secondNode->getNumChildren() == 2,
           "hiding the parent node should suppress both child branches without removing them");

    osg::observer_ptr<osg::Node> secondNodeObserver(secondNode);
    osg::observer_ptr<osg::Node> secondBoundsObserver(
        secondNode ? secondNode->getChild(1) : nullptr);
    scene.removePotreeNode("r1");
    expect(scene.pointCount() == 2,
           "removing a Potree node should update resident point count");
    expect(!secondNodeObserver.valid() && !secondBoundsObserver.valid(),
           "removing the parent should release its point and bounding-box subtree");
    scene.clear();
    expect(scene.root()->getNumChildren() == 0,
           "multi-node Potree scene should clear cleanly");
}

void testCesiumCameraMatrixContract()
{
    osg::ref_ptr<CesiumCameraManipulator> manipulator = new CesiumCameraManipulator;

    const osg::Vec3d eye(12.0, -8.0, 5.0);
    const osg::Vec3d center(-2.0, 4.0, 1.0);
    const osg::Vec3d up(0.0, 0.0, 1.0);
    const osg::Matrixd viewMatrix = osg::Matrixd::lookAt(eye, center, up);
    const osg::Matrixd cameraMatrix = osg::Matrixd::inverse(viewMatrix);

    manipulator->setByMatrix(cameraMatrix);
    expect(nearMatrix(manipulator->getMatrix(), cameraMatrix),
           "setByMatrix/getMatrix should preserve a rigid camera matrix");
    expect(nearMatrix(manipulator->getInverseMatrix(), viewMatrix),
           "getInverseMatrix should return the matching view matrix");
    expect(nearMatrix(manipulator->getMatrix() * manipulator->getInverseMatrix(),
                      osg::Matrixd::identity()),
           "camera and view matrices should be mutual inverses");

    manipulator->setByInverseMatrix(viewMatrix);
    expect(nearMatrix(manipulator->getMatrix(), cameraMatrix),
           "setByInverseMatrix should recover camera position and orientation");

    const osg::Matrixd beforeRoundTrip = manipulator->getMatrix();
    manipulator->setByMatrix(manipulator->getMatrix());
    expect(nearMatrix(manipulator->getMatrix(), beforeRoundTrip),
           "setByMatrix(getMatrix()) should not move or rotate the camera");
}

void testCameraControllerMatrixTransfer()
{
    const osg::Vec3d eye(12.0, -8.0, 5.0);
    const osg::Vec3d center(-2.0, 4.0, 1.0);
    const osg::Matrixd cameraMatrix = osg::Matrixd::inverse(
        osg::Matrixd::lookAt(eye, center, osg::Vec3d(0.0, 0.0, 1.0)));
    const osg::Quat rotation = cameraMatrix.getRotate();
    const double focusDistance = (center - eye).length();

    osg::ref_ptr<CesiumCameraManipulator> cesium =
        new CesiumCameraManipulator;
    cesium->setByMatrix(cameraMatrix);
    cesium->setFocusDistance(focusDistance);

    osg::ref_ptr<osgGA::TrackballManipulator> trackball =
        new osgGA::TrackballManipulator;
    trackball->setTransformation(
        eye,
        eye + rotation * osg::Vec3d(0.0, 0.0, -focusDistance),
        rotation * osg::Vec3d(0.0, 1.0, 0.0));
    expect(nearMatrix(trackball->getMatrix(), cameraMatrix),
           "switching to Trackball should preserve the camera matrix");

    trackball->setDistance(focusDistance * 0.5);
    const osg::Matrixd trackballMatrix = trackball->getMatrix();
    cesium->setByMatrix(trackballMatrix);
    cesium->setFocusDistance(trackball->getDistance());
    expect(nearMatrix(cesium->getMatrix(), trackballMatrix)
               && nearDouble(cesium->focusDistance(), focusDistance * 0.5),
           "switching back to Cesium should preserve matrix and focus distance");

    const double validDistance = cesium->focusDistance();
    cesium->setFocusDistance(0.0);
    expect(nearDouble(cesium->focusDistance(), validDistance),
           "controller transfer should reject an invalid focus distance");
}

void testCesiumCameraHomeAndNodeContract()
{
    osg::ref_ptr<osg::Geode> scene = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(-4.0f, 1.0f, 2.0f));
    vertices->push_back(osg::Vec3(8.0f, 1.0f, 2.0f));
    vertices->push_back(osg::Vec3(2.0f, -5.0f, 2.0f));
    vertices->push_back(osg::Vec3(2.0f, 7.0f, 2.0f));
    vertices->push_back(osg::Vec3(2.0f, 1.0f, -4.0f));
    vertices->push_back(osg::Vec3(2.0f, 1.0f, 8.0f));
    geometry->setVertexArray(vertices.get());
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size()));
    scene->addDrawable(geometry.get());

    osg::ref_ptr<CesiumCameraManipulator> manipulator = new CesiumCameraManipulator;
    manipulator->setNode(scene.get());
    expect(manipulator->getNode() == scene.get(),
           "setNode/getNode should preserve the attached scene node");
    const CesiumCameraManipulator* constManipulator = manipulator.get();
    expect(constManipulator->getNode() == scene.get(),
           "const getNode should return the attached scene node");

    manipulator->home(0.0);
    const osg::Matrixd firstHome = manipulator->getMatrix();
    const osg::BoundingSphere bound = scene->getBound();
    const osg::Vec3d homeEye = firstHome.getTrans();
    const double homeDistance = (bound.center() - homeEye).length();
    const osg::Vec3d pointOnForwardAxis = osg::Vec3d(0.0, 0.0, -homeDistance)
        * firstHome;
    expect((pointOnForwardAxis - bound.center()).length() < 1.0e-8,
           "home should aim the camera local -Z axis at the scene bound center");
    expect(homeDistance + 1.0e-9 >= static_cast<double>(bound.radius()) * 4.0,
           "home(double) should use the conservative fallback framing distance");

    manipulator->setByMatrix(osg::Matrixd::translate(100.0, 200.0, 300.0));
    manipulator->home(1.0);
    expect(nearMatrix(manipulator->getMatrix(), firstHome),
           "repeated home(double) calls should be deterministic");

    osg::ref_ptr<osgGA::GUIEventAdapter> event = new osgGA::GUIEventAdapter;
    TestGuiActionAdapter action;
    manipulator->setByMatrix(osg::Matrixd::translate(-10.0, -20.0, -30.0));
    manipulator->home(*event, action);
    expect(nearMatrix(manipulator->getMatrix(), firstHome),
           "event home overload should produce the same fallback home pose");
    expect(action.redrawRequested && !action.continuousUpdateRequested,
           "event home should request one redraw and stop continuous updates");

    osg::ref_ptr<osg::View> view = new osg::View;
    view->getCamera()->setProjectionMatrixAsPerspective(30.0, 16.0 / 9.0, 0.1, 10000.0);
    action.view = view.get();
    manipulator->home(*event, action);
    const double perspectiveHomeDistance =
        (bound.center() - manipulator->getMatrix().getTrans()).length();
    const double expectedPerspectiveDistance = static_cast<double>(bound.radius())
        / std::sin(osg::DegreesToRadians(15.0)) * 1.05;
    const double perspectiveDistanceTolerance =
        std::max(1.0, expectedPerspectiveDistance) * 1.0e-6;
    expect(std::abs(perspectiveHomeDistance - expectedPerspectiveDistance)
               < perspectiveDistanceTolerance,
           "event home should frame the scene using the active perspective FOV");

    osg::ref_ptr<osg::Group> emptyScene = new osg::Group;
    manipulator->setNode(emptyScene.get());
    const osg::Matrixd poseBeforeInvalidHome = manipulator->getMatrix();
    manipulator->home(2.0);
    expect(nearMatrix(manipulator->getMatrix(), poseBeforeInvalidHome),
           "home should preserve a valid pose when the scene bound is invalid");
}

void testCameraFramebufferCoordinatesAndRays()
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setViewport(0, 0, 800, 600);

    const osg::Vec3d eye(0.0, -10.0, 2.0);
    const osg::Vec3d center(0.0, 0.0, 0.0);
    camera->setViewMatrixAsLookAt(eye, center, osg::Vec3d(0.0, 0.0, 1.0));
    camera->setProjectionMatrixAsPerspective(60.0, 4.0 / 3.0, 0.1, 1000.0);

    osg::ref_ptr<osgGA::GUIEventAdapter> event = new osgGA::GUIEventAdapter;
    event->setWindowRectangle(0, 0, 800, 600);
    int pixelX = -1;
    int pixelY = -1;

    event->setMouseYOrientation(osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);
    event->setX(0.0f);
    event->setY(0.0f);
    expect(CameraMath::eventToFramebufferPixel(*event, *camera, pixelX, pixelY)
               && pixelX == 0 && pixelY == 0,
           "upward OSG event origin should map to the framebuffer origin");

    event->setX(799.0f);
    event->setY(599.0f);
    expect(CameraMath::eventToFramebufferPixel(*event, *camera, pixelX, pixelY)
               && pixelX == 799 && pixelY == 599,
           "upward OSG event top-right should preserve framebuffer Y");

    event->setMouseYOrientation(osgGA::GUIEventAdapter::Y_INCREASING_DOWNWARDS);
    event->setX(0.0f);
    event->setY(0.0f);
    expect(CameraMath::eventToFramebufferPixel(*event, *camera, pixelX, pixelY)
               && pixelX == 0 && pixelY == 599,
           "downward OSG event top-left should map to top-left framebuffer pixel");

    event->setX(799.0f);
    event->setY(599.0f);
    expect(CameraMath::eventToFramebufferPixel(*event, *camera, pixelX, pixelY)
               && pixelX == 799 && pixelY == 0,
           "downward OSG event bottom-right should map to framebuffer Y zero");

    event->setX(800.0f);
    expect(!CameraMath::eventToFramebufferPixel(*event, *camera, pixelX, pixelY),
           "events outside the physical viewport should be rejected");

    const osg::Vec3d forward = (center - eye) / (center - eye).length();
    const osg::Matrixd cameraMatrix = osg::Matrixd::inverse(camera->getViewMatrix());
    const osg::Quat cameraRotation = cameraMatrix.getRotate();
    const osg::Vec3d right = cameraRotation * osg::Vec3d(1.0, 0.0, 0.0);
    const osg::Vec3d up = cameraRotation * osg::Vec3d(0.0, 1.0, 0.0);

    event->setMouseYOrientation(osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);
    event->setX(400.0f);
    event->setY(300.0f);
    osg::Vec3d rayOrigin;
    osg::Vec3d rayDirection;
    expect(CameraMath::buildPerspectiveMouseRay(
               *event, *camera, eye, rayOrigin, rayDirection)
               && (rayOrigin - eye).length() < 1.0e-12
               && rayDirection * forward > 1.0 - 1.0e-10,
           "center pixel ray should start at the perspective eye and follow camera forward");

    event->setX(0.0f);
    event->setY(599.0f);
    expect(CameraMath::buildPerspectiveMouseRay(
               *event, *camera, eye, rayOrigin, rayDirection)
               && rayDirection * right < 0.0
               && rayDirection * up > 0.0,
           "top-left ray should point left and up in camera space");

    struct PhysicalViewport
    {
        int width;
        int height;
    };
    const PhysicalViewport dpiViewports[] = {{1000, 750}, {1200, 900}};
    for (const PhysicalViewport viewport : dpiViewports) {
        camera->setViewport(0, 0, viewport.width, viewport.height);
        event->setWindowRectangle(0, 0, viewport.width, viewport.height);
        camera->setProjectionMatrixAsPerspective(
            60.0,
            static_cast<double>(viewport.width) / viewport.height,
            0.1,
            1000.0);
        event->setX(static_cast<float>(viewport.width / 2));
        event->setY(static_cast<float>(viewport.height / 2));
        expect(CameraMath::buildPerspectiveMouseRay(
                   *event, *camera, eye, rayOrigin, rayDirection)
                   && rayDirection * forward > 1.0 - 1.0e-10,
               "physical center ray should not apply devicePixelRatio a second time");
    }
}

void testCameraProjectionRoundTripAndNearFar()
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setViewport(0, 0, 1280, 720);
    camera->setViewMatrixAsLookAt(
        osg::Vec3d(4.0, -12.0, 6.0),
        osg::Vec3d(1.0, 2.0, 0.5),
        osg::Vec3d(0.0, 0.0, 1.0));
    camera->setProjectionMatrixAsPerspective(45.0, 16.0 / 9.0, 0.01, 5000.0);

    const osg::Vec3d worldPoint(1.5, 3.0, 0.75);
    osg::Vec3d framebufferPoint;
    osg::Vec3d reconstructedPoint;
    expect(CameraMath::projectWorldToFramebuffer(
               worldPoint, *camera, framebufferPoint)
               && framebufferPoint.z() > 0.0 && framebufferPoint.z() < 1.0
               && CameraMath::unprojectFramebufferPoint(
                   framebufferPoint.x(),
                   framebufferPoint.y(),
                   framebufferPoint.z(),
                   *camera,
                   reconstructedPoint)
               && (reconstructedPoint - worldPoint).length() < 1.0e-8,
           "projecting and unprojecting should recover the original world point");

    const osg::BoundingSphere bound(osg::Vec3d(0.0, 0.0, 0.0), 10.0f);
    double nearPlane = 0.0;
    double farPlane = 0.0;
    expect(CameraMath::computePerspectiveNearFar(
               osg::Vec3d(0.0, -100.0, 0.0),
               bound,
               10.0,
               nearPlane,
               farPlane)
               && nearDouble(nearPlane, 89.0)
               && nearDouble(farPlane, 111.0),
           "near/far should include a padded scene bound when camera is outside it");

    expect(CameraMath::computePerspectiveNearFar(
               osg::Vec3d(0.0, 0.0, 0.0),
               bound,
               10.0,
               nearPlane,
               farPlane)
               && nearPlane > 0.0
               && nearDouble(nearPlane, 0.01)
               && nearDouble(farPlane, 11.0),
           "camera inside the scene bound should derive near from focus distance");

    expect(CameraMath::computePerspectiveNearFar(
               osg::Vec3d(0.0, 0.0, 0.0),
               bound,
               0.01,
               nearPlane,
               farPlane)
               && nearDouble(nearPlane, 1.0e-5)
               && nearDouble(farPlane, 11.0),
           "near should shrink with focus distance for close detail views");

    expect(!CameraMath::computePerspectiveNearFar(
               osg::Vec3d(0.0, 0.0, 0.0),
               bound,
               0.0,
               nearPlane,
               farPlane),
           "near/far should reject a non-positive focus distance");
}

void testCameraInteractionMath()
{
    double zoomDistance = 0.0;
    expect(CameraMath::computeExponentialZoomDistance(
               100.0, 1.0, 0.15, 0.1, 1000000.0, zoomDistance)
               && nearDouble(zoomDistance, 85.0),
           "one forward wheel step should apply the configured exponential zoom scale");
    expect(CameraMath::computeExponentialZoomDistance(
               100.0, -1.0, 0.15, 0.1, 1000000.0, zoomDistance)
               && nearDouble(zoomDistance, 100.0 / 0.85),
           "one backward wheel step should apply the reciprocal zoom scale");

    double batchedDistance = 0.0;
    expect(CameraMath::computeExponentialZoomDistance(
               100.0, 3.0, 0.15, 0.1, 1000000.0, batchedDistance),
           "batched exponential zoom should accept finite wheel steps");
    double incrementalDistance = 100.0;
    for (int step = 0; step < 3; ++step) {
        double nextDistance = 0.0;
        expect(CameraMath::computeExponentialZoomDistance(
                   incrementalDistance,
                   1.0,
                   0.15,
                   0.1,
                   1000000.0,
                   nextDistance),
               "incremental exponential zoom step should remain valid");
        incrementalDistance = nextDistance;
    }
    expect(std::abs(batchedDistance - incrementalDistance) < 1.0e-10,
           "batched and repeated wheel steps should produce the same distance");

    expect(CameraMath::computeExponentialZoomDistance(
               0.11, 10.0, 0.15, 0.1, 1000000.0, zoomDistance)
               && nearDouble(zoomDistance, 0.1),
           "zoom-in distance should clamp at the minimum without crossing the pivot");
    expect(CameraMath::computeExponentialZoomDistance(
               999999.0, -10.0, 0.15, 0.1, 1000000.0, zoomDistance)
               && nearDouble(zoomDistance, 1000000.0),
           "zoom-out distance should clamp at the maximum");

    osg::Vec3d intersection;
    expect(CameraMath::intersectRayWithPlane(
               osg::Vec3d(1.0, 2.0, 0.0),
               osg::Vec3d(0.0, 0.0, 2.0),
               osg::Vec3d(0.0, 0.0, 5.0),
               osg::Vec3d(0.0, 0.0, 1.0),
               intersection)
               && (intersection - osg::Vec3d(1.0, 2.0, 5.0)).length() < 1.0e-12,
           "forward ray should intersect the fixed mathematical plane");
    expect(!CameraMath::intersectRayWithPlane(
               osg::Vec3d(),
               osg::Vec3d(1.0, 0.0, 0.0),
               osg::Vec3d(0.0, 0.0, 5.0),
               osg::Vec3d(0.0, 0.0, 1.0),
               intersection),
           "parallel ray should not produce a plane intersection");
    expect(!CameraMath::intersectRayWithPlane(
               osg::Vec3d(),
               osg::Vec3d(0.0, 0.0, 1.0),
               osg::Vec3d(0.0, 0.0, -1.0),
               osg::Vec3d(0.0, 0.0, 1.0),
               intersection),
           "plane behind the ray origin should be rejected");

    const double minimumPitch = osg::DegreesToRadians(-89.0);
    const double maximumPitch = osg::DegreesToRadians(89.0);
    double allowedPitch = 0.0;
    expect(CameraMath::clampPitchDelta(
               osg::Vec3d(0.0, 1.0, 0.0),
               osg::Vec3d(0.0, 0.0, 1.0),
               osg::DegreesToRadians(100.0),
               minimumPitch,
               maximumPitch,
               allowedPitch)
               && std::abs(allowedPitch - maximumPitch) < 1.0e-12,
           "pitch request should clamp to the positive 89 degree limit");

    const osg::Vec3d forwardAtEightyDegrees(
        0.0,
        std::cos(osg::DegreesToRadians(80.0)),
        std::sin(osg::DegreesToRadians(80.0)));
    expect(CameraMath::clampPitchDelta(
               forwardAtEightyDegrees,
               osg::Vec3d(0.0, 0.0, 1.0),
               osg::DegreesToRadians(20.0),
               minimumPitch,
               maximumPitch,
               allowedPitch)
               && std::abs(allowedPitch - osg::DegreesToRadians(9.0)) < 1.0e-12,
           "pitch clamp should derive the remaining motion from the current forward vector");
}

void testCesiumCameraImmediateInteractions()
{
    osg::ref_ptr<osg::View> view = new osg::View;
    osg::Camera* camera = view->getCamera();
    camera->setViewport(0, 0, 800, 600);
    camera->setProjectionMatrixAsPerspective(60.0, 4.0 / 3.0, 0.1, 1000.0);

    TestGuiActionAdapter action;
    action.view = view.get();
    osg::ref_ptr<osgGA::GUIEventAdapter> event = new osgGA::GUIEventAdapter;
    event->setWindowRectangle(0, 0, 800, 600);
    event->setMouseYOrientation(osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);
    event->setX(400.0f);
    event->setY(300.0f);

    osg::ref_ptr<CesiumCameraManipulator> zoomManipulator =
        new CesiumCameraManipulator;
    const osg::Quat zoomRotationBefore = zoomManipulator->getMatrix().getRotate();
    event->setEventType(osgGA::GUIEventAdapter::SCROLL);
    event->setScrollingMotion(osgGA::GUIEventAdapter::SCROLL_UP);
    expect(zoomManipulator->handle(*event, action),
           "discrete scroll-up should be consumed by the camera manipulator");
    expect((zoomManipulator->getMatrix().getTrans()
            - osg::Vec3d(0.0, -8.5, 0.0)).length() < 1.0e-10,
           "center scroll-up should move the eye by the 0.85 exponential scale");
    const osg::Quat zoomRotationAfter = zoomManipulator->getMatrix().getRotate();
    expect((zoomRotationBefore * osg::Vec3d(0.0, 0.0, -1.0)
            - zoomRotationAfter * osg::Vec3d(0.0, 0.0, -1.0)).length()
               < 1.0e-12,
           "zoom should preserve camera orientation");
    expect(action.redrawRequested && !action.continuousUpdateRequested,
           "Immediate zoom should request one redraw without continuous updates");

    for (int step = 0; step < 100; ++step) {
        zoomManipulator->handle(*event, action);
    }
    const osg::Vec3d minimumZoomEye = zoomManipulator->getMatrix().getTrans();
    expect(minimumZoomEye.y() < 0.0
               && std::abs(minimumZoomEye.y() + 0.01) < 1.0e-9,
           "repeated scroll-up should stop at the minimum distance without crossing the pivot");
    event->setScrollingMotion(osgGA::GUIEventAdapter::SCROLL_DOWN);
    expect(zoomManipulator->handle(*event, action)
               && zoomManipulator->getMatrix().getTrans().y()
                   < minimumZoomEye.y(),
           "scroll-down should move away from the same default pivot");

    osg::ref_ptr<CesiumCameraManipulator> panManipulator =
        new CesiumCameraManipulator;
    event->setEventType(osgGA::GUIEventAdapter::PUSH);
    event->setButton(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setButtonMask(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setX(400.0f);
    event->setY(300.0f);
    const osg::Matrixd beforePanPush = panManipulator->getMatrix();
    expect(panManipulator->handle(*event, action)
               && nearMatrix(panManipulator->getMatrix(), beforePanPush),
           "left-button press should establish the pan plane without moving the camera");

    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(500.0f);
    expect(panManipulator->handle(*event, action),
           "left-button drag should update Immediate pan");
    const osg::Matrixd afterPanDrag = panManipulator->getMatrix();
    const osg::Vec3d forwardBeforePan = beforePanPush.getRotate()
        * osg::Vec3d(0.0, 0.0, -1.0);
    const osg::Vec3d forwardAfterPan = afterPanDrag.getRotate()
        * osg::Vec3d(0.0, 0.0, -1.0);
    expect((afterPanDrag.getTrans() - beforePanPush.getTrans()).length() > 0.1
               && (forwardAfterPan - forwardBeforePan).length() < 1.0e-12,
           "pan should translate the camera without changing its orientation");

    event->setEventType(osgGA::GUIEventAdapter::RELEASE);
    event->setButton(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setButtonMask(0);
    expect(panManipulator->handle(*event, action),
           "left-button release should finish the Immediate pan gesture");
    const osg::Matrixd afterPanRelease = panManipulator->getMatrix();
    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setButtonMask(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setX(600.0f);
    expect(!panManipulator->handle(*event, action)
               && nearMatrix(panManipulator->getMatrix(), afterPanRelease),
           "drag after release should not continue moving the camera");

    osg::ref_ptr<CesiumCameraManipulator> rotateManipulator =
        new CesiumCameraManipulator;
    event->setEventType(osgGA::GUIEventAdapter::PUSH);
    event->setButton(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
    event->setButtonMask(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
    event->setX(400.0f);
    event->setY(300.0f);
    const osg::Matrixd beforeRotatePush = rotateManipulator->getMatrix();
    expect(rotateManipulator->handle(*event, action)
               && nearMatrix(rotateManipulator->getMatrix(), beforeRotatePush),
           "middle-button press should capture the pivot without a camera jump");

    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(500.0f);
    expect(rotateManipulator->handle(*event, action),
           "middle-button horizontal drag should apply Immediate yaw");
    osg::Matrixd rotatedMatrix = rotateManipulator->getMatrix();
    osg::Vec3d rotatedEye = rotatedMatrix.getTrans();
    osg::Vec3d rotatedForward = rotatedMatrix.getRotate()
        * osg::Vec3d(0.0, 0.0, -1.0);
    expect(std::abs(rotatedEye.length() - 10.0) < 1.0e-9
               && rotatedForward * (-rotatedEye / rotatedEye.length())
                   > 1.0 - 1.0e-12,
           "yaw should orbit at a fixed radius while continuing to look at the pivot");

    event->setY(-10000.0f);
    expect(rotateManipulator->handle(*event, action),
           "large pitch drag should be handled and clamped");
    rotatedMatrix = rotateManipulator->getMatrix();
    rotatedForward = rotatedMatrix.getRotate()
        * osg::Vec3d(0.0, 0.0, -1.0);
    osg::Vec3d rotatedRight = rotatedMatrix.getRotate()
        * osg::Vec3d(1.0, 0.0, 0.0);
    const double positivePitch = std::asin(std::clamp(
        rotatedForward.z() / rotatedForward.length(), -1.0, 1.0));
    expect(std::abs(positivePitch - osg::DegreesToRadians(89.0)) < 1.0e-9,
           "rotation should clamp positive pitch to 89 degrees");
    expect(std::abs(rotatedRight * osg::Vec3d(0.0, 0.0, 1.0)) < 1.0e-10,
           "rotation should rebuild a roll-free right vector");

    event->setY(10000.0f);
    expect(rotateManipulator->handle(*event, action),
           "opposite large pitch drag should remain valid");
    rotatedMatrix = rotateManipulator->getMatrix();
    rotatedForward = rotatedMatrix.getRotate()
        * osg::Vec3d(0.0, 0.0, -1.0);
    rotatedRight = rotatedMatrix.getRotate()
        * osg::Vec3d(1.0, 0.0, 0.0);
    const double negativePitch = std::asin(std::clamp(
        rotatedForward.z() / rotatedForward.length(), -1.0, 1.0));
    expect(std::abs(negativePitch - osg::DegreesToRadians(-89.0)) < 1.0e-9,
           "rotation should clamp negative pitch to -89 degrees");
    expect(std::abs(rotatedRight * osg::Vec3d(0.0, 0.0, 1.0)) < 1.0e-10,
           "repeated extreme rotation should not accumulate camera roll");
}

void testCesiumCameraDepthDrivenInteractions()
{
    osg::ref_ptr<osg::View> view = new osg::View;
    osg::Camera* camera = view->getCamera();
    camera->setViewport(0, 0, 800, 600);
    camera->setProjectionMatrixAsPerspective(
        60.0, 4.0 / 3.0, 0.1, 1000.0);

    TestGuiActionAdapter action;
    action.view = view.get();
    osg::ref_ptr<osgGA::GUIEventAdapter> event =
        new osgGA::GUIEventAdapter;
    event->setWindowRectangle(0, 0, 800, 600);
    event->setMouseYOrientation(
        osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);

    osg::ref_ptr<CesiumCameraManipulator> zoomManipulator =
        new CesiumCameraManipulator;
    osg::ref_ptr<TestDepthBufferPicker> zoomPicker =
        new TestDepthBufferPicker;
    osg::ref_ptr<PickDebugVisualizer> zoomVisualizer =
        new PickDebugVisualizer;
    zoomManipulator->setDepthBufferPicker(zoomPicker.get());
    zoomManipulator->setPickDebugVisualizer(zoomVisualizer.get());
    zoomManipulator->setPickDebugVisible(true);

    event->setEventType(osgGA::GUIEventAdapter::SCROLL);
    event->setScrollingMotion(osgGA::GUIEventAdapter::SCROLL_UP);
    event->setX(400.0f);
    event->setY(300.0f);
    const osg::Matrixd beforePickedZoom = zoomManipulator->getMatrix();
    expect(zoomManipulator->handle(*event, action)
               && nearMatrix(zoomManipulator->getMatrix(), beforePickedZoom),
           "depth-driven zoom should wait for the PostDraw result");
    event->setX(500.0f);
    expect(zoomManipulator->handle(*event, action)
               && nearMatrix(zoomManipulator->getMatrix(), beforePickedZoom),
           "a second wheel event should merge while depth is pending");

    DepthPickRequest request;
    expect(zoomPicker->takeRequest(request)
               && request.action == PickAction::Zoom
               && nearDouble(request.wheelSteps, 2.0)
               && request.pixelX == 500,
           "rapid discrete wheel input should keep latest coordinates and cumulative steps");
    const osg::Vec3d zoomPivot(3.0, 1.0, 0.0);
    DepthPickResult result;
    result.generation = request.generation;
    result.sequence = request.sequence;
    result.action = request.action;
    result.wheelSteps = request.wheelSteps;
    result.hitScene = true;
    result.worldPoint = zoomPivot;
    zoomPicker->emitResult(result);

    event->setEventType(osgGA::GUIEventAdapter::FRAME);
    zoomManipulator->handle(*event, action);
    const double pickedZoomScale = std::pow(0.85, 2.0);
    const osg::Vec3d expectedPickedZoomEye = zoomPivot
        + (beforePickedZoom.getTrans() - zoomPivot) * pickedZoomScale;
    expect((zoomManipulator->getMatrix().getTrans()
            - expectedPickedZoomEye).length() < 1.0e-9
               && zoomVisualizer->hasMarker()
               && (zoomVisualizer->worldPoint() - zoomPivot).length()
                   < 1.0e-12,
           "valid depth should be the exponential zoom pivot and debug marker position");

    event->setEventType(osgGA::GUIEventAdapter::SCROLL);
    event->setScrollingMotion(osgGA::GUIEventAdapter::SCROLL_UP);
    event->setX(200.0f);
    event->setY(300.0f);
    const osg::Matrixd beforeMissZoom = zoomManipulator->getMatrix();
    expect(zoomManipulator->handle(*event, action)
               && zoomPicker->takeRequest(request),
           "background zoom should still submit a depth request");
    osg::Vec3d missRayOrigin;
    osg::Vec3d missRayDirection;
    expect(CameraMath::buildPerspectiveMouseRay(
               *event,
               *camera,
               zoomManipulator->getInverseMatrix(),
               beforeMissZoom.getTrans(),
               missRayOrigin,
               missRayDirection),
           "background zoom test should reconstruct the current mouse ray");
    result.generation = request.generation;
    result.sequence = request.sequence;
    result.action = request.action;
    result.wheelSteps = request.wheelSteps;
    result.hitScene = false;
    zoomPicker->emitResult(result);
    event->setEventType(osgGA::GUIEventAdapter::FRAME);
    zoomManipulator->handle(*event, action);
    const double focusDistanceAfterPickedZoom =
        (beforePickedZoom.getTrans() - zoomPivot).length()
        * pickedZoomScale;
    missRayDirection.normalize();
    const osg::Vec3d expectedMissZoomEye = beforeMissZoom.getTrans()
        + missRayDirection * focusDistanceAfterPickedZoom * 0.15;
    expect((zoomManipulator->getMatrix().getTrans()
            - expectedMissZoomEye).length() < 1.0e-9
               && !zoomVisualizer->hasMarker(),
           "depth miss should use the current mouse ray and clear the old marker");

    osg::ref_ptr<CesiumCameraManipulator> panManipulator =
        new CesiumCameraManipulator;
    osg::ref_ptr<TestDepthBufferPicker> panPicker =
        new TestDepthBufferPicker;
    panManipulator->setDepthBufferPicker(panPicker.get());
    event->setEventType(osgGA::GUIEventAdapter::PUSH);
    event->setButton(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setButtonMask(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setX(400.0f);
    event->setY(300.0f);
    const osg::Matrixd beforePickedPan = panManipulator->getMatrix();
    osg::ref_ptr<osgGA::GUIEventAdapter> panPress =
        new osgGA::GUIEventAdapter(*event);
    expect(panManipulator->handle(*event, action)
               && panPicker->takeRequest(request),
           "left press should request a fixed pan-plane point");
    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(500.0f);
    osg::ref_ptr<osgGA::GUIEventAdapter> panDrag =
        new osgGA::GUIEventAdapter(*event);
    expect(panManipulator->handle(*event, action)
               && nearMatrix(panManipulator->getMatrix(), beforePickedPan),
           "pan should cache the latest drag without moving before depth resolves");

    const osg::Vec3d panPlanePoint(2.0, 2.0, 0.0);
    osg::Vec3d pressRayOrigin;
    osg::Vec3d pressRayDirection;
    osg::Vec3d dragRayOrigin;
    osg::Vec3d dragRayDirection;
    osg::Vec3d pressPlanePoint;
    osg::Vec3d dragPlanePoint;
    const osg::Vec3d panPlaneNormal(0.0, 1.0, 0.0);
    expect(CameraMath::buildPerspectiveMouseRay(
               *panPress,
               *camera,
               panManipulator->getInverseMatrix(),
               beforePickedPan.getTrans(),
               pressRayOrigin,
               pressRayDirection)
               && CameraMath::buildPerspectiveMouseRay(
                   *panDrag,
                   *camera,
                   panManipulator->getInverseMatrix(),
                   beforePickedPan.getTrans(),
                   dragRayOrigin,
                   dragRayDirection)
               && CameraMath::intersectRayWithPlane(
                   pressRayOrigin,
                   pressRayDirection,
                   panPlanePoint,
                   panPlaneNormal,
                   pressPlanePoint)
               && CameraMath::intersectRayWithPlane(
                   dragRayOrigin,
                   dragRayDirection,
                   panPlanePoint,
                   panPlaneNormal,
                   dragPlanePoint),
           "picked pan plane should support press/latest-drag intersection");
    result.generation = request.generation;
    result.sequence = request.sequence;
    result.action = request.action;
    result.wheelSteps = 0.0;
    result.hitScene = true;
    result.worldPoint = panPlanePoint;
    panPicker->emitResult(result);
    event->setEventType(osgGA::GUIEventAdapter::FRAME);
    panManipulator->handle(*event, action);
    const osg::Vec3d expectedPanEye = beforePickedPan.getTrans()
        + pressPlanePoint - dragPlanePoint;
    expect((panManipulator->getMatrix().getTrans() - expectedPanEye).length()
                   < 1.0e-9
               && (panManipulator->getMatrix().getRotate()
                   * osg::Vec3d(0.0, 0.0, -1.0)
                   - beforePickedPan.getRotate()
                       * osg::Vec3d(0.0, 0.0, -1.0)).length()
                   < 1.0e-12,
           "resolved pan should apply cached motion on the picked fixed plane without rotating");

    event->setEventType(osgGA::GUIEventAdapter::RELEASE);
    event->setButton(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setButtonMask(0);
    expect(panManipulator->handle(*event, action),
           "resolved pan should finish normally on release");

    osg::ref_ptr<CesiumCameraManipulator> rotateManipulator =
        new CesiumCameraManipulator;
    osg::ref_ptr<TestDepthBufferPicker> rotatePicker =
        new TestDepthBufferPicker;
    rotateManipulator->setDepthBufferPicker(rotatePicker.get());
    event->setEventType(osgGA::GUIEventAdapter::PUSH);
    event->setButton(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
    event->setButtonMask(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
    event->setX(400.0f);
    event->setY(300.0f);
    const osg::Matrixd beforePickedRotate = rotateManipulator->getMatrix();
    expect(rotateManipulator->handle(*event, action)
               && rotatePicker->takeRequest(request),
           "middle press should request a fixed rotation pivot");
    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(500.0f);
    expect(rotateManipulator->handle(*event, action)
               && nearMatrix(rotateManipulator->getMatrix(), beforePickedRotate),
           "rotate should cache drag motion without moving before depth resolves");
    const osg::Vec3d rotationPivot(2.0, 1.0, 0.0);
    result.generation = request.generation;
    result.sequence = request.sequence;
    result.action = request.action;
    result.hitScene = true;
    result.worldPoint = rotationPivot;
    rotatePicker->emitResult(result);
    event->setEventType(osgGA::GUIEventAdapter::FRAME);
    rotateManipulator->handle(*event, action);
    const osg::Vec3d pickedRotateEye =
        rotateManipulator->getMatrix().getTrans();
    const double initialPivotDistance =
        (beforePickedRotate.getTrans() - rotationPivot).length();
    expect(!nearMatrix(rotateManipulator->getMatrix(), beforePickedRotate)
               && std::abs((pickedRotateEye - rotationPivot).length()
                           - initialPivotDistance) < 1.0e-9,
           "resolved rotate should apply cached motion around the picked fixed pivot");

    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButtonMask(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
    event->setX(550.0f);
    expect(rotateManipulator->handle(*event, action)
               && std::abs((rotateManipulator->getMatrix().getTrans()
                            - rotationPivot).length()
                           - initialPivotDistance) < 1.0e-9,
           "subsequent rotate drag should retain the same pivot for the gesture");

    const double yawOnlyEyeHeight =
        (rotateManipulator->getMatrix().getTrans() - rotationPivot)
            * osg::Vec3d(0.0, 0.0, 1.0);
    const double yawOnlyForwardHeight =
        (rotateManipulator->getMatrix().getRotate()
         * osg::Vec3d(0.0, 0.0, -1.0))
            * osg::Vec3d(0.0, 0.0, 1.0);
    bool yawStayedOnWorldUp = true;
    for (int step = 1; step <= 200; ++step) {
        event->setX(550.0f + static_cast<float>(step));
        event->setY(300.0f);
        if (!rotateManipulator->handle(*event, action)) {
            yawStayedOnWorldUp = false;
            break;
        }
        const osg::Matrixd yawMatrix = rotateManipulator->getMatrix();
        const double eyeHeight =
            (yawMatrix.getTrans() - rotationPivot)
                * osg::Vec3d(0.0, 0.0, 1.0);
        const double forwardHeight =
            (yawMatrix.getRotate() * osg::Vec3d(0.0, 0.0, -1.0))
                * osg::Vec3d(0.0, 0.0, 1.0);
        if (std::abs(eyeHeight - yawOnlyEyeHeight) > 1.0e-9
            || std::abs(forwardHeight - yawOnlyForwardHeight) > 1.0e-9) {
            yawStayedOnWorldUp = false;
            break;
        }
    }
    expect(yawStayedOnWorldUp,
           "horizontal-only drag should remain a pure worldUp yaw after many increments");

    osg::ref_ptr<CesiumCameraManipulator> missPanManipulator =
        new CesiumCameraManipulator;
    osg::ref_ptr<TestDepthBufferPicker> missPanPicker =
        new TestDepthBufferPicker;
    missPanManipulator->setDepthBufferPicker(missPanPicker.get());
    event->setEventType(osgGA::GUIEventAdapter::PUSH);
    event->setButton(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setButtonMask(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setX(400.0f);
    event->setY(300.0f);
    const osg::Matrixd beforeMissPan = missPanManipulator->getMatrix();
    expect(missPanManipulator->handle(*event, action)
               && missPanPicker->takeRequest(request),
           "pan miss test should submit its press request");
    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(450.0f);
    missPanManipulator->handle(*event, action);
    result.generation = request.generation;
    result.sequence = request.sequence;
    result.action = request.action;
    result.hitScene = false;
    missPanPicker->emitResult(result);
    event->setEventType(osgGA::GUIEventAdapter::FRAME);
    missPanManipulator->handle(*event, action);
    expect(!nearMatrix(missPanManipulator->getMatrix(), beforeMissPan)
               && nearMatrix(osg::Matrixd::rotate(
                                 missPanManipulator->getMatrix().getRotate()),
                             osg::Matrixd::rotate(beforeMissPan.getRotate())),
           "pan depth miss should fall back to the focusPoint view plane");

    osg::ref_ptr<CesiumCameraManipulator> missRotateManipulator =
        new CesiumCameraManipulator;
    osg::ref_ptr<TestDepthBufferPicker> missRotatePicker =
        new TestDepthBufferPicker;
    missRotateManipulator->setDepthBufferPicker(missRotatePicker.get());
    event->setEventType(osgGA::GUIEventAdapter::PUSH);
    event->setButton(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
    event->setButtonMask(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
    event->setX(400.0f);
    event->setY(300.0f);
    const osg::Matrixd beforeMissRotate = missRotateManipulator->getMatrix();
    expect(missRotateManipulator->handle(*event, action)
               && missRotatePicker->takeRequest(request),
           "rotate miss test should submit its press request");
    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(450.0f);
    missRotateManipulator->handle(*event, action);
    result.generation = request.generation;
    result.sequence = request.sequence;
    result.action = request.action;
    result.hitScene = false;
    missRotatePicker->emitResult(result);
    event->setEventType(osgGA::GUIEventAdapter::FRAME);
    missRotateManipulator->handle(*event, action);
    expect(!nearMatrix(missRotateManipulator->getMatrix(), beforeMissRotate)
               && std::abs(missRotateManipulator->getMatrix().getTrans().length()
                           - 10.0) < 1.0e-9,
           "rotate depth miss should fall back to the current focusPoint pivot");

    osg::ref_ptr<CesiumCameraManipulator> staleManipulator =
        new CesiumCameraManipulator;
    osg::ref_ptr<TestDepthBufferPicker> stalePicker =
        new TestDepthBufferPicker;
    osg::ref_ptr<PickDebugVisualizer> staleVisualizer =
        new PickDebugVisualizer;
    staleManipulator->setDepthBufferPicker(stalePicker.get());
    staleManipulator->setPickDebugVisualizer(staleVisualizer.get());
    event->setEventType(osgGA::GUIEventAdapter::PUSH);
    event->setButton(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setButtonMask(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setX(400.0f);
    event->setY(300.0f);
    const osg::Matrixd beforeStaleGesture = staleManipulator->getMatrix();
    expect(staleManipulator->handle(*event, action)
               && stalePicker->takeRequest(request),
           "stale-result test should take the request as if PostDraw had begun");
    event->setEventType(osgGA::GUIEventAdapter::RELEASE);
    event->setButton(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    event->setButtonMask(0);
    expect(staleManipulator->handle(*event, action),
           "release before depth completion should end the pending gesture");
    result.generation = request.generation;
    result.sequence = request.sequence;
    result.action = request.action;
    result.hitScene = true;
    result.worldPoint = osg::Vec3d(50.0, 50.0, 50.0);
    stalePicker->emitResult(result);
    event->setEventType(osgGA::GUIEventAdapter::FRAME);
    staleManipulator->handle(*event, action);
    expect(nearMatrix(staleManipulator->getMatrix(), beforeStaleGesture)
               && !staleVisualizer->hasMarker(),
           "result arriving after release should not move the camera or update the marker");
}

void testCesiumCameraRotationInputFilters()
{
    osg::ref_ptr<osg::View> view = new osg::View;
    osg::Camera* camera = view->getCamera();
    camera->setViewport(0, 0, 800, 600);
    camera->setProjectionMatrixAsPerspective(
        60.0, 4.0 / 3.0, 0.1, 1000.0);

    TestGuiActionAdapter action;
    action.view = view.get();
    osg::ref_ptr<osgGA::GUIEventAdapter> event =
        new osgGA::GUIEventAdapter;
    event->setWindowRectangle(0, 0, 800, 600);
    event->setMouseYOrientation(
        osgGA::GUIEventAdapter::Y_INCREASING_UPWARDS);

    auto beginRotate = [&](CesiumCameraManipulator& manipulator) {
        event->setEventType(osgGA::GUIEventAdapter::PUSH);
        event->setButton(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
        event->setButtonMask(osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
        event->setX(400.0f);
        event->setY(300.0f);
        return manipulator.handle(*event, action);
    };

    osg::ref_ptr<CesiumCameraManipulator> ignoreVertical =
        new CesiumCameraManipulator;
    expect(!ignoreVertical->ignoreHorizontalRotationInput()
               && !ignoreVertical->ignoreVerticalRotationInput(),
           "rotation input debug filters should be disabled by default");
    ignoreVertical->setIgnoreVerticalRotationInput(true);
    expect(beginRotate(*ignoreVertical),
           "vertical-filter test should begin rotation");
    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(500.0f);
    event->setY(450.0f);
    expect(ignoreVertical->handle(*event, action),
           "vertical-filter test should consume mixed-axis drag");
    const osg::Matrixd yawOnlyMatrix = ignoreVertical->getMatrix();
    const osg::Vec3d yawOnlyForward = yawOnlyMatrix.getRotate()
        * osg::Vec3d(0.0, 0.0, -1.0);
    expect(std::abs(yawOnlyMatrix.getTrans().z()) < 1.0e-12
               && std::abs(yawOnlyForward.z()) < 1.0e-12
               && std::abs(yawOnlyMatrix.getTrans().x()) > 0.1,
           "ignoring vertical mouse input should leave only worldUp yaw");

    osg::ref_ptr<CesiumCameraManipulator> ignoreHorizontal =
        new CesiumCameraManipulator;
    ignoreHorizontal->setIgnoreHorizontalRotationInput(true);
    expect(beginRotate(*ignoreHorizontal),
           "horizontal-filter test should begin rotation");
    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(500.0f);
    event->setY(350.0f);
    expect(ignoreHorizontal->handle(*event, action),
           "horizontal-filter test should consume mixed-axis drag");
    const osg::Matrixd pitchOnlyMatrix = ignoreHorizontal->getMatrix();
    const osg::Vec3d pitchOnlyForward = pitchOnlyMatrix.getRotate()
        * osg::Vec3d(0.0, 0.0, -1.0);
    expect(std::abs(pitchOnlyMatrix.getTrans().x()) < 1.0e-12
               && std::abs(pitchOnlyForward.x()) < 1.0e-12
               && std::abs(pitchOnlyMatrix.getTrans().z()) > 0.1,
           "ignoring horizontal mouse input should leave only camera-right pitch");

    osg::ref_ptr<CesiumCameraManipulator> ignoreBoth =
        new CesiumCameraManipulator;
    ignoreBoth->setIgnoreHorizontalRotationInput(true);
    ignoreBoth->setIgnoreVerticalRotationInput(true);
    expect(beginRotate(*ignoreBoth),
           "dual-filter test should begin rotation");
    const osg::Matrixd beforeIgnoredDrag = ignoreBoth->getMatrix();
    event->setEventType(osgGA::GUIEventAdapter::DRAG);
    event->setButton(0);
    event->setX(500.0f);
    event->setY(400.0f);
    expect(ignoreBoth->handle(*event, action)
               && nearMatrix(ignoreBoth->getMatrix(), beforeIgnoredDrag),
           "enabling both filters should suppress all rotation motion");
}

void testDepthPickerSelectionAndInvalidation()
{
    osg::ref_ptr<osg::Viewport> viewport =
        new osg::Viewport(0.0, 0.0, 10.0, 8.0);
    DepthReadRegion region;
    expect(DepthBufferPicker::computeReadRegion(5, 4, *viewport, region)
               && region.x == 3 && region.y == 2
               && region.width == 5 && region.height == 5,
           "depth picker should read a centered 5x5 region inside the viewport");
    expect(DepthBufferPicker::computeReadRegion(0, 0, *viewport, region)
               && region.x == 0 && region.y == 0
               && region.width == 3 && region.height == 3,
           "depth picker should clip its read region at the lower-left viewport edge");
    expect(DepthBufferPicker::computeReadRegion(9, 7, *viewport, region)
               && region.x == 7 && region.y == 5
               && region.width == 3 && region.height == 3,
           "depth picker should clip its read region at the upper-right viewport edge");
    expect(!DepthBufferPicker::computeReadRegion(10, 7, *viewport, region),
           "depth picker should reject requests outside the viewport");

    region = {3, 2, 5, 5};
    std::array<float, 25> depths;
    depths.fill(1.0f);
    depths[2 * 5 + 0] = 0.1f;
    depths[1 * 5 + 2] = 0.7f;
    DepthSampleSelection selection;
    expect(DepthBufferPicker::selectNearestValidDepth(
               depths.data(), depths.size(), region, 5, 4, selection)
               && selection.pixelX == 5 && selection.pixelY == 3
               && std::abs(selection.depth - 0.7f) < 1.0e-7f,
           "5x5 picker should choose the valid sample nearest the requested pixel");

    depths[2 * 5 + 2] = 0.9f;
    expect(DepthBufferPicker::selectNearestValidDepth(
               depths.data(), depths.size(), region, 5, 4, selection)
               && selection.pixelX == 5 && selection.pixelY == 4
               && std::abs(selection.depth - 0.9f) < 1.0e-7f,
           "center valid depth should win even when a farther sample is shallower");

    depths.fill(1.0f);
    depths[0] = 0.0f;
    depths[1] = std::numeric_limits<float>::quiet_NaN();
    expect(!DepthBufferPicker::selectNearestValidDepth(
               depths.data(), depths.size(), region, 5, 4, selection),
           "zero, non-finite and cleared depths should produce a background miss");

    depths.fill(1.0f);
    depths[2 * 5 + 2] = std::nextafter(1.0f, 0.0f);
    expect(DepthBufferPicker::selectNearestValidDepth(
               depths.data(), depths.size(), region, 5, 4, selection)
               && selection.pixelX == 5 && selection.pixelY == 4
               && selection.depth < 1.0f,
           "visible depth immediately below the clear value should remain pickable");

    DepthPickResult result;
    result.generation = 7;
    result.sequence = 12;
    expect(DepthBufferPicker::isResultCurrent(result, 7, 12),
           "matching generation and sequence should accept a depth result");
    expect(!DepthBufferPicker::isResultCurrent(result, 8, 12),
           "scene generation change should invalidate an old depth result");
    expect(!DepthBufferPicker::isResultCurrent(result, 7, 13),
           "newer request sequence should invalidate an old depth result");

    osg::ref_ptr<DepthBufferPicker> picker = new DepthBufferPicker;
    DepthPickRequest request;
    request.generation = 1;
    request.sequence = 1;
    picker->requestPick(request);
    picker->clear();
    expect(!picker->consumeResult(result),
           "clearing the picker should discard queued mailbox state");
}

void testPickDebugVisualizerState()
{
    osg::ref_ptr<PickDebugVisualizer> visualizer = new PickDebugVisualizer;
    expect(!visualizer->visible() && !visualizer->hasMarker(),
           "pick debug visualizer should be hidden and empty by default");
    expect(visualizer->node()->getNodeMask() == PotreeRenderMasks::PickDebug,
           "pick marker should use its independent debug node mask");

    osg::Switch* markerSwitch = dynamic_cast<osg::Switch*>(visualizer->node());
    osg::AutoTransform* markerTransform = markerSwitch
        ? dynamic_cast<osg::AutoTransform*>(markerSwitch->getChild(0))
        : nullptr;
    expect(markerSwitch && markerTransform
               && markerTransform->getAutoScaleToScreen()
               && !markerSwitch->getValue(0),
           "pick marker should use a hidden fixed-screen-size transform");

    const osg::StateSet* stateSet = visualizer->node()->getStateSet();
    const osg::Depth* depth = stateSet
        ? dynamic_cast<const osg::Depth*>(
            stateSet->getAttribute(osg::StateAttribute::DEPTH))
        : nullptr;
    expect(depth && depth->getFunction() == osg::Depth::ALWAYS
               && !depth->getWriteMask(),
           "pick marker should always be visible without writing depth");

    visualizer->setVisible(true);
    visualizer->show(osg::Vec3d(1.0, 2.0, 3.0), PickAction::Zoom);
    expect(visualizer->hasMarker() && markerSwitch->getValue(0)
               && (visualizer->worldPoint() - osg::Vec3d(1.0, 2.0, 3.0)).length()
                   < 1.0e-12
               && visualizer->color() == osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f),
           "Zoom pick should display a yellow marker at the world position");

    visualizer->show(osg::Vec3d(4.0, 5.0, 6.0), PickAction::BeginPan);
    expect(visualizer->color() == osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f),
           "Pan pick should display a cyan marker");
    visualizer->show(osg::Vec3d(7.0, 8.0, 9.0), PickAction::BeginRotate);
    expect(visualizer->color() == osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f),
           "Rotate pick should display a magenta marker");

    osg::ref_ptr<osg::Group> mainScene = new osg::Group;
    osg::ref_ptr<osg::Geode> mainGeode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> mainGeometry = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> mainVertices = new osg::Vec3Array;
    mainVertices->push_back(osg::Vec3(-1.0f, -1.0f, -1.0f));
    mainVertices->push_back(osg::Vec3(1.0f, 1.0f, 1.0f));
    mainGeometry->setVertexArray(mainVertices.get());
    mainGeometry->addPrimitiveSet(
        new osg::DrawArrays(GL_POINTS, 0, mainVertices->size()));
    mainGeode->addDrawable(mainGeometry.get());
    mainScene->addChild(mainGeode.get());
    const osg::BoundingSphere mainBoundBefore = mainScene->getBound();
    osg::ref_ptr<osg::Camera> separateDebugCamera = new osg::Camera;
    separateDebugCamera->addChild(visualizer->node());
    const osg::BoundingSphere mainBoundAfter = mainScene->getBound();
    expect((mainBoundAfter.center() - mainBoundBefore.center()).length() < 1.0e-12
               && std::abs(mainBoundAfter.radius() - mainBoundBefore.radius())
                   < 1.0e-12,
           "marker attached to a separate debug camera should not affect the main scene bound");

    visualizer->clear();
    expect(!visualizer->hasMarker() && !markerSwitch->getValue(0),
           "failed or cleared pick should hide the previous marker");
}

void testProjectionUpdateRunsAfterViewUpdate()
{
    osg::ref_ptr<CesiumCameraManipulator> manipulator = new CesiumCameraManipulator;
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    const osg::Matrixd expectedView = osg::Matrixd::lookAt(
        osg::Vec3d(5.0, -7.0, 3.0),
        osg::Vec3d(0.0, 0.0, 0.0),
        osg::Vec3d(0.0, 0.0, 1.0));
    manipulator->setByInverseMatrix(expectedView);

    bool callbackRan = false;
    bool callbackSawUpdatedView = false;
    manipulator->setPostViewUpdateCallback(
        [&](osg::Camera& updatedCamera) {
            callbackRan = true;
            callbackSawUpdatedView = nearMatrix(
                updatedCamera.getViewMatrix(), expectedView);
        });
    manipulator->updateCamera(*camera);

    expect(callbackRan && callbackSawUpdatedView,
           "projection hook should run only after manipulator writes the current view matrix");
}
} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    testValidDefaultDataset();
    testPlySceneRegression();
    testMissingHierarchy();
    testInvalidHierarchyChunkSize();
    testOctreeRangeAndEncodingErrors();
    testProxyHierarchyPatch();
    testLocalSampleFullHierarchyStats();
    testLodProjectionAndSelection();
    testLodFrustumAndRootFallback();
    testLodBudgetAndRequests();
    testLodPixelThresholdAndProxy();
    testNodeLoadSchedulerCompletionQueue();
    testPotreeSceneMultipleNodes();
    testCesiumCameraMatrixContract();
    testCameraControllerMatrixTransfer();
    testCesiumCameraHomeAndNodeContract();
    testCameraFramebufferCoordinatesAndRays();
    testCameraProjectionRoundTripAndNearFar();
    testCameraInteractionMath();
    testCesiumCameraImmediateInteractions();
    testCesiumCameraDepthDrivenInteractions();
    testCesiumCameraRotationInputFilters();
    testDepthPickerSelectionAndInvalidation();
    testPickDebugVisualizerState();
    testProjectionUpdateRunsAfterViewUpdate();

    if (failures == 0) {
        std::cout << "All point-cloud tests passed.\n";
    }
    return failures == 0 ? 0 : 1;
}
