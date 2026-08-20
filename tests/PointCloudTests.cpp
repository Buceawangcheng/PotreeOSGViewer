#include "pointcloud/Potree2Provider.h"
#include "viewer/SceneManager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QtEndian>

#include <cmath>
#include <cstdint>
#include <iostream>

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
                           std::uint64_t size)
{
    QByteArray bytes(22, '\0');
    uchar* output = reinterpret_cast<uchar*>(bytes.data());
    output[0] = type;
    output[1] = 0;
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
    expect(dataset->root->loadState == OctreeNodeLoadState::Unloaded,
           "hierarchy parsing should not mark point data ready");

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
    expect(dataset->root->loadState == OctreeNodeLoadState::CpuReady,
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
} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    testValidDefaultDataset();
    testPlySceneRegression();
    testMissingHierarchy();
    testInvalidHierarchyChunkSize();
    testOctreeRangeAndEncodingErrors();

    if (failures == 0) {
        std::cout << "All point-cloud tests passed.\n";
    }
    return failures == 0 ? 0 : 1;
}
