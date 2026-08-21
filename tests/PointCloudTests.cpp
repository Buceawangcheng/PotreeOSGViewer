#include "pointcloud/Potree2Provider.h"
#include "pointcloud/LodSelector.h"
#include "pointcloud/NodeLoadScheduler.h"
#include "viewer/PotreeRenderMasks.h"
#include "viewer/SceneManager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QThread>
#include <QtEndian>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Program>
#include <osg/observer_ptr>

#include <cmath>
#include <cstdint>
#include <iostream>
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

    if (failures == 0) {
        std::cout << "All point-cloud tests passed.\n";
    }
    return failures == 0 ? 0 : 1;
}
