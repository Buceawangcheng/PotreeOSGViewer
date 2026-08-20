#include "pointcloud/Potree2Provider.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QtEndian>

#include <algorithm>
#include <limits>
#include <vector>

namespace
{
constexpr int HierarchyRecordSize = 22;

std::uint8_t colorComponent(std::uint16_t value)
{
    return static_cast<std::uint8_t>(value > 255 ? value / 256 : value);
}

bool attributeOffset(const PointAttributes& attributes,
                     std::string_view name,
                     std::uint32_t* offset,
                     const PointAttribute** result)
{
    std::uint32_t currentOffset = 0;
    for (const PointAttribute& attribute : attributes.items()) {
        if (attribute.name == name) {
            *offset = currentOffset;
            *result = &attribute;
            return true;
        }
        currentOffset += attribute.sizeBytes;
    }

    return false;
}

QString missingFieldError(const QString& fieldName, const QString& path)
{
    return QStringLiteral("Missing or invalid field '%1' in %2.").arg(fieldName, path);
}

bool readRequiredObject(const QJsonObject& object,
                        const QString& fieldName,
                        const QString& path,
                        QJsonObject* value,
                        QString* errorMessage)
{
    const QJsonValue jsonValue = object.value(fieldName);
    if (!jsonValue.isObject()) {
        if (errorMessage) {
            *errorMessage = missingFieldError(fieldName, path);
        }
        return false;
    }

    *value = jsonValue.toObject();
    return true;
}

bool readRequiredString(const QJsonObject& object,
                        const QString& fieldName,
                        const QString& path,
                        QString* value,
                        QString* errorMessage)
{
    const QJsonValue jsonValue = object.value(fieldName);
    if (!jsonValue.isString()) {
        if (errorMessage) {
            *errorMessage = missingFieldError(fieldName, path);
        }
        return false;
    }

    *value = jsonValue.toString();
    return true;
}

bool readRequiredDouble(const QJsonObject& object,
                        const QString& fieldName,
                        const QString& path,
                        double* value,
                        QString* errorMessage)
{
    const QJsonValue jsonValue = object.value(fieldName);
    if (!jsonValue.isDouble()) {
        if (errorMessage) {
            *errorMessage = missingFieldError(fieldName, path);
        }
        return false;
    }

    *value = jsonValue.toDouble();
    return true;
}

bool readRequiredUInt64(const QJsonObject& object,
                        const QString& fieldName,
                        const QString& path,
                        std::uint64_t* value,
                        QString* errorMessage)
{
    const QJsonValue jsonValue = object.value(fieldName);
    if (!jsonValue.isDouble()) {
        if (errorMessage) {
            *errorMessage = missingFieldError(fieldName, path);
        }
        return false;
    }

    const double number = jsonValue.toDouble();
    if (number < 0.0 || number > static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
        if (errorMessage) {
            *errorMessage = missingFieldError(fieldName, path);
        }
        return false;
    }

    *value = static_cast<std::uint64_t>(number);
    return true;
}

bool readRequiredUInt32(const QJsonObject& object,
                        const QString& fieldName,
                        const QString& path,
                        std::uint32_t* value,
                        QString* errorMessage)
{
    std::uint64_t integer = 0;
    if (!readRequiredUInt64(object, fieldName, path, &integer, errorMessage)
        || integer > std::numeric_limits<std::uint32_t>::max()) {
        if (errorMessage && errorMessage->isEmpty()) {
            *errorMessage = missingFieldError(fieldName, path);
        }
        return false;
    }

    *value = static_cast<std::uint32_t>(integer);
    return true;
}

bool readDoubleArray(const QJsonObject& object,
                     const QString& fieldName,
                     const QString& path,
                     std::vector<double>* values,
                     QString* errorMessage)
{
    const QJsonValue jsonValue = object.value(fieldName);
    if (!jsonValue.isArray()) {
        if (errorMessage) {
            *errorMessage = missingFieldError(fieldName, path);
        }
        return false;
    }

    const QJsonArray array = jsonValue.toArray();
    values->clear();
    values->reserve(array.size());
    for (const QJsonValue& item : array) {
        if (!item.isDouble()) {
            if (errorMessage) {
                *errorMessage = missingFieldError(fieldName, path);
            }
            return false;
        }
        values->push_back(item.toDouble());
    }

    return true;
}

bool readVec3d(const QJsonObject& object,
               const QString& fieldName,
               const QString& path,
               osg::Vec3d* value,
               QString* errorMessage)
{
    std::vector<double> values;
    if (!readDoubleArray(object, fieldName, path, &values, errorMessage) || values.size() != 3) {
        if (errorMessage && errorMessage->isEmpty()) {
            *errorMessage = missingFieldError(fieldName, path);
        }
        return false;
    }

    value->set(values[0], values[1], values[2]);
    return true;
}

PointAttributeType pointAttributeTypeFromString(const QString& typeName)
{
    if (typeName == QLatin1String("int8")) {
        return PointAttributeType::Int8;
    }
    if (typeName == QLatin1String("uint8")) {
        return PointAttributeType::UInt8;
    }
    if (typeName == QLatin1String("int16")) {
        return PointAttributeType::Int16;
    }
    if (typeName == QLatin1String("uint16")) {
        return PointAttributeType::UInt16;
    }
    if (typeName == QLatin1String("int32")) {
        return PointAttributeType::Int32;
    }
    if (typeName == QLatin1String("uint32")) {
        return PointAttributeType::UInt32;
    }
    if (typeName == QLatin1String("int64")) {
        return PointAttributeType::Int64;
    }
    if (typeName == QLatin1String("uint64")) {
        return PointAttributeType::UInt64;
    }
    if (typeName == QLatin1String("float")) {
        return PointAttributeType::Float;
    }
    if (typeName == QLatin1String("double")) {
        return PointAttributeType::Double;
    }

    return PointAttributeType::Unknown;
}

bool octreeNodeTypeFromRecord(std::uint8_t type, OctreeNodeType* nodeType)
{
    switch (type) {
    case 0:
        *nodeType = OctreeNodeType::Normal;
        return true;
    case 1:
        *nodeType = OctreeNodeType::Leaf;
        return true;
    case 2:
        *nodeType = OctreeNodeType::Proxy;
        return true;
    default:
        return false;
    }
}

BoundingBox childBounds(const BoundingBox& parent, int childIndex)
{
    const osg::Vec3d center = (parent.min + parent.max) * 0.5;
    BoundingBox child = parent;

    if ((childIndex & 0b100) != 0) {
        child.min.x() = center.x();
    } else {
        child.max.x() = center.x();
    }

    if ((childIndex & 0b010) != 0) {
        child.min.y() = center.y();
    } else {
        child.max.y() = center.y();
    }

    if ((childIndex & 0b001) != 0) {
        child.min.z() = center.z();
    } else {
        child.max.z() = center.z();
    }

    return child;
}

std::uint32_t maxLoadedLevel(const OctreeNode& node)
{
    std::uint32_t level = node.level;
    for (const std::unique_ptr<OctreeNode>& child : node.children) {
        if (child) {
            level = std::max(level, maxLoadedLevel(*child));
        }
    }

    return level;
}
} // namespace

bool Potree2Provider::canOpen(const QString& path) const
{
    QString metadataPath;
    QString datasetDir;
    return resolveDatasetPaths(path, &metadataPath, &datasetDir, nullptr);
}

std::shared_ptr<PointCloudDataset> Potree2Provider::openMetadata(const QString& path,
                                                                 QString* errorMessage)
{
    if (errorMessage) {
        errorMessage->clear();
    }

    QString metadataPath;
    QString datasetDir;
    if (!resolveDatasetPaths(path, &metadataPath, &datasetDir, errorMessage)) {
        return nullptr;
    }

    PotreeMetadata metadata;
    if (!readMetadata(metadataPath, datasetDir, &metadata, errorMessage)) {
        return nullptr;
    }

    auto dataset = std::make_shared<PointCloudDataset>();
    dataset->sourcePath = datasetDir;
    dataset->name = metadata.name;
    dataset->format = QStringLiteral("Potree 2.0");
    dataset->version = metadata.version;
    dataset->encoding = metadata.encoding;
    dataset->totalPoints = metadata.points;
    dataset->spacing = metadata.spacing;
    dataset->offset = metadata.offset;
    dataset->scale = metadata.scale;
    dataset->bounds = metadata.bounds;
    dataset->attributes = std::move(metadata.attributes);

    if (!parseFirstHierarchyChunk(metadata, dataset.get(), errorMessage)) {
        return nullptr;
    }

    return dataset;
}

std::shared_ptr<PointCloudNodeData> Potree2Provider::loadNodeData(
    const PointCloudDataset& dataset,
    OctreeNode* node,
    QString* errorMessage) const
{
    if (errorMessage) {
        errorMessage->clear();
    }

    if (!node) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot load a null octree node.");
        }
        return nullptr;
    }

    if (dataset.encoding != QLatin1String("DEFAULT")) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Point data decoding for Potree encoding '%1' is not supported yet.")
                                .arg(dataset.encoding);
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    if (node->type == OctreeNodeType::Proxy) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Node '%1' is a hierarchy proxy and has no point-data range.")
                                .arg(QString::fromStdString(node->id));
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    const std::uint32_t recordSize = dataset.attributes.pointRecordSizeBytes();
    if (recordSize == 0
        || node->pointCount > std::numeric_limits<std::uint64_t>::max() / recordSize
        || node->pointCount * recordSize != node->byteSize) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Node '%1' byte size does not match its point count and metadata record size.")
                                .arg(QString::fromStdString(node->id));
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    std::uint32_t positionOffset = 0;
    const PointAttribute* positionAttribute = nullptr;
    if (!attributeOffset(dataset.attributes,
                         "position",
                         &positionOffset,
                         &positionAttribute)
        || positionAttribute->type != PointAttributeType::Int32
        || positionAttribute->numElements != 3
        || positionAttribute->sizeBytes != 12) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("DEFAULT decoding requires a 3-component int32 'position' attribute.");
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    std::uint32_t colorOffset = 0;
    const PointAttribute* colorAttribute = nullptr;
    const bool hasColor = attributeOffset(dataset.attributes,
                                          "rgb",
                                          &colorOffset,
                                          &colorAttribute);
    if (hasColor
        && (colorAttribute->type != PointAttributeType::UInt16
            || colorAttribute->numElements != 3
            || colorAttribute->sizeBytes != 6)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("DEFAULT decoding requires 'rgb' to contain three uint16 components.");
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    if (node->byteOffset > static_cast<std::uint64_t>(std::numeric_limits<qint64>::max())
        || node->byteSize > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Node '%1' point-data range is too large for this synchronous loader.")
                                .arg(QString::fromStdString(node->id));
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    const QString octreePath = QDir(dataset.sourcePath).filePath(QStringLiteral("octree.bin"));
    QFile octreeFile(octreePath);
    if (!octreeFile.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to open octree.bin:\n%1").arg(octreePath);
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    const std::uint64_t fileSize = static_cast<std::uint64_t>(octreeFile.size());
    if (node->byteOffset > fileSize || node->byteSize > fileSize - node->byteOffset) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Node '%1' point-data range lies outside %2.")
                                .arg(QString::fromStdString(node->id), octreePath);
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    node->loadState = OctreeNodeLoadState::Loading;
    if (!octreeFile.seek(static_cast<qint64>(node->byteOffset))) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to seek to node '%1' in %2.")
                                .arg(QString::fromStdString(node->id), octreePath);
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    const QByteArray encoded = octreeFile.read(static_cast<qint64>(node->byteSize));
    if (encoded.size() != static_cast<int>(node->byteSize)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to read node '%1' from %2. Expected %3 bytes, got %4 bytes.")
                                .arg(QString::fromStdString(node->id), octreePath)
                                .arg(node->byteSize)
                                .arg(encoded.size());
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    if (node->pointCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Node '%1' contains too many points for this process.")
                                .arg(QString::fromStdString(node->id));
        }
        node->loadState = OctreeNodeLoadState::Failed;
        return nullptr;
    }

    auto data = std::make_shared<PointCloudNodeData>();
    data->origin = node->bounds.min;
    data->positions.resize(static_cast<std::size_t>(node->pointCount));
    data->colors.resize(static_cast<std::size_t>(node->pointCount));

    const uchar* bytes = reinterpret_cast<const uchar*>(encoded.constData());
    for (std::uint64_t pointIndex = 0; pointIndex < node->pointCount; ++pointIndex) {
        const uchar* record = bytes + (pointIndex * recordSize);
        const std::int32_t x = qFromLittleEndian<std::int32_t>(record + positionOffset);
        const std::int32_t y = qFromLittleEndian<std::int32_t>(record + positionOffset + 4);
        const std::int32_t z = qFromLittleEndian<std::int32_t>(record + positionOffset + 8);

        const osg::Vec3d worldPosition(
            static_cast<double>(x) * dataset.scale.x() + dataset.offset.x(),
            static_cast<double>(y) * dataset.scale.y() + dataset.offset.y(),
            static_cast<double>(z) * dataset.scale.z() + dataset.offset.z());
        const osg::Vec3d localPosition = worldPosition - data->origin;
        data->positions[static_cast<std::size_t>(pointIndex)].set(
            static_cast<float>(localPosition.x()),
            static_cast<float>(localPosition.y()),
            static_cast<float>(localPosition.z()));

        osg::Vec4ub color(255, 255, 255, 255);
        if (hasColor) {
            const std::uint16_t r = qFromLittleEndian<std::uint16_t>(record + colorOffset);
            const std::uint16_t g = qFromLittleEndian<std::uint16_t>(record + colorOffset + 2);
            const std::uint16_t b = qFromLittleEndian<std::uint16_t>(record + colorOffset + 4);
            color.set(colorComponent(r), colorComponent(g), colorComponent(b), 255);
        }
        data->colors[static_cast<std::size_t>(pointIndex)] = color;
    }

    node->data = data;
    node->cpuBytes = data->cpuBytes();
    node->loadState = OctreeNodeLoadState::CpuReady;
    return data;
}

bool Potree2Provider::resolveDatasetPaths(const QString& path,
                                          QString* metadataPath,
                                          QString* datasetDir,
                                          QString* errorMessage) const
{
    const QFileInfo inputInfo(path);
    QString resolvedMetadataPath;
    QString resolvedDatasetDir;

    if (inputInfo.isDir()) {
        resolvedDatasetDir = inputInfo.absoluteFilePath();
        resolvedMetadataPath = QDir(resolvedDatasetDir).filePath(QStringLiteral("metadata.json"));
    } else if (inputInfo.isFile() && inputInfo.fileName() == QLatin1String("metadata.json")) {
        resolvedMetadataPath = inputInfo.absoluteFilePath();
        resolvedDatasetDir = inputInfo.absolutePath();
    } else {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Expected a Potree 2.x directory or metadata.json file:\n%1")
                                .arg(path);
        }
        return false;
    }

    if (!QFileInfo::exists(resolvedMetadataPath)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Missing metadata.json:\n%1").arg(resolvedMetadataPath);
        }
        return false;
    }

    const QString hierarchyPath = QDir(resolvedDatasetDir).filePath(QStringLiteral("hierarchy.bin"));
    if (!QFileInfo::exists(hierarchyPath)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Missing hierarchy.bin:\n%1").arg(hierarchyPath);
        }
        return false;
    }

    const QString octreePath = QDir(resolvedDatasetDir).filePath(QStringLiteral("octree.bin"));
    if (!QFileInfo::exists(octreePath)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Missing octree.bin:\n%1").arg(octreePath);
        }
        return false;
    }

    if (metadataPath) {
        *metadataPath = resolvedMetadataPath;
    }
    if (datasetDir) {
        *datasetDir = resolvedDatasetDir;
    }

    return true;
}

bool Potree2Provider::readMetadata(const QString& metadataPath,
                                   const QString& datasetDir,
                                   PotreeMetadata* metadata,
                                   QString* errorMessage) const
{
    QFile metadataFile(metadataPath);
    if (!metadataFile.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to open metadata.json:\n%1").arg(metadataPath);
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(metadataFile.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Invalid JSON in %1:\n%2")
                                .arg(metadataPath, parseError.errorString());
        }
        return false;
    }

    const QJsonObject root = document.object();
    metadata->metadataPath = metadataPath;
    metadata->datasetDir = datasetDir;

    if (!readRequiredString(root, QStringLiteral("version"), metadataPath, &metadata->version, errorMessage)
        || !readRequiredString(root, QStringLiteral("name"), metadataPath, &metadata->name, errorMessage)
        || !readRequiredUInt64(root, QStringLiteral("points"), metadataPath, &metadata->points, errorMessage)
        || !readRequiredString(root, QStringLiteral("encoding"), metadataPath, &metadata->encoding, errorMessage)
        || !readRequiredDouble(root, QStringLiteral("spacing"), metadataPath, &metadata->spacing, errorMessage)
        || !readVec3d(root, QStringLiteral("offset"), metadataPath, &metadata->offset, errorMessage)
        || !readVec3d(root, QStringLiteral("scale"), metadataPath, &metadata->scale, errorMessage)) {
        return false;
    }

    if (!metadata->version.startsWith(QLatin1String("2."))) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Unsupported Potree metadata version '%1' in %2.")
                                .arg(metadata->version, metadataPath);
        }
        return false;
    }

    if (metadata->encoding != QLatin1String("DEFAULT")
        && metadata->encoding != QLatin1String("BROTLI")) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Unsupported Potree encoding '%1' in %2.")
                                .arg(metadata->encoding, metadataPath);
        }
        return false;
    }

    QJsonObject hierarchy;
    if (!readRequiredObject(root, QStringLiteral("hierarchy"), metadataPath, &hierarchy, errorMessage)
        || !readRequiredUInt64(hierarchy,
                               QStringLiteral("firstChunkSize"),
                               metadataPath,
                               &metadata->hierarchy.firstChunkSize,
                               errorMessage)
        || !readRequiredUInt32(hierarchy,
                               QStringLiteral("stepSize"),
                               metadataPath,
                               &metadata->hierarchy.stepSize,
                               errorMessage)
        || !readRequiredUInt32(hierarchy,
                               QStringLiteral("depth"),
                               metadataPath,
                               &metadata->hierarchy.depth,
                               errorMessage)) {
        return false;
    }

    QJsonObject boundingBox;
    if (!readRequiredObject(root, QStringLiteral("boundingBox"), metadataPath, &boundingBox, errorMessage)
        || !readVec3d(boundingBox, QStringLiteral("min"), metadataPath, &metadata->bounds.min, errorMessage)
        || !readVec3d(boundingBox, QStringLiteral("max"), metadataPath, &metadata->bounds.max, errorMessage)) {
        return false;
    }

    const QJsonValue attributesValue = root.value(QStringLiteral("attributes"));
    if (!attributesValue.isArray()) {
        if (errorMessage) {
            *errorMessage = missingFieldError(QStringLiteral("attributes"), metadataPath);
        }
        return false;
    }

    const QJsonArray attributes = attributesValue.toArray();
    for (int i = 0; i < attributes.size(); ++i) {
        const QString attributePath = QStringLiteral("%1.attributes[%2]").arg(metadataPath).arg(i);
        if (!attributes.at(i).isObject()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Invalid attribute entry in %1.").arg(attributePath);
            }
            return false;
        }

        const QJsonObject attributeObject = attributes.at(i).toObject();
        QString name;
        QString description;
        QString typeName;
        std::uint32_t size = 0;
        std::uint32_t numElements = 0;
        std::uint32_t elementSize = 0;

        if (!readRequiredString(attributeObject, QStringLiteral("name"), attributePath, &name, errorMessage)
            || !readRequiredString(attributeObject,
                                  QStringLiteral("description"),
                                  attributePath,
                                  &description,
                                  errorMessage)
            || !readRequiredUInt32(attributeObject, QStringLiteral("size"), attributePath, &size, errorMessage)
            || !readRequiredUInt32(attributeObject,
                                  QStringLiteral("numElements"),
                                  attributePath,
                                  &numElements,
                                  errorMessage)
            || !readRequiredUInt32(attributeObject,
                                  QStringLiteral("elementSize"),
                                  attributePath,
                                  &elementSize,
                                  errorMessage)
            || !readRequiredString(attributeObject, QStringLiteral("type"), attributePath, &typeName, errorMessage)) {
            return false;
        }

        PointAttribute attribute;
        attribute.name = name.toStdString();
        attribute.description = description.toStdString();
        attribute.sizeBytes = size;
        attribute.numElements = numElements;
        attribute.elementSizeBytes = elementSize;
        attribute.type = pointAttributeTypeFromString(typeName);

        if (attributeObject.contains(QStringLiteral("min"))
            && !readDoubleArray(attributeObject, QStringLiteral("min"), attributePath, &attribute.min, errorMessage)) {
            return false;
        }
        if (attributeObject.contains(QStringLiteral("max"))
            && !readDoubleArray(attributeObject, QStringLiteral("max"), attributePath, &attribute.max, errorMessage)) {
            return false;
        }
        if (attributeObject.contains(QStringLiteral("scale"))
            && !readDoubleArray(attributeObject,
                                QStringLiteral("scale"),
                                attributePath,
                                &attribute.scale,
                                errorMessage)) {
            return false;
        }
        if (attributeObject.contains(QStringLiteral("offset"))
            && !readDoubleArray(attributeObject,
                                QStringLiteral("offset"),
                                attributePath,
                                &attribute.offset,
                                errorMessage)) {
            return false;
        }

        metadata->attributes.add(std::move(attribute));
    }

    return true;
}

bool Potree2Provider::parseFirstHierarchyChunk(const PotreeMetadata& metadata,
                                               PointCloudDataset* dataset,
                                               QString* errorMessage) const
{
    if (metadata.hierarchy.firstChunkSize == 0
        || metadata.hierarchy.firstChunkSize > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Invalid hierarchy.firstChunkSize in %1.")
                                .arg(metadata.metadataPath);
        }
        return false;
    }

    const QString hierarchyPath = QDir(metadata.datasetDir).filePath(QStringLiteral("hierarchy.bin"));
    QFile hierarchyFile(hierarchyPath);
    if (!hierarchyFile.open(QIODevice::ReadOnly)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to open hierarchy.bin:\n%1").arg(hierarchyPath);
        }
        return false;
    }

    const QByteArray chunk = hierarchyFile.read(static_cast<qint64>(metadata.hierarchy.firstChunkSize));
    if (chunk.size() != static_cast<int>(metadata.hierarchy.firstChunkSize)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Failed to read first hierarchy chunk from %1. Expected %2 bytes, got %3 bytes.")
                                .arg(hierarchyPath)
                                .arg(metadata.hierarchy.firstChunkSize)
                                .arg(chunk.size());
        }
        return false;
    }

    if ((chunk.size() % HierarchyRecordSize) != 0) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Invalid first hierarchy chunk size in %1: %2 is not divisible by %3.")
                                .arg(hierarchyPath)
                                .arg(chunk.size())
                                .arg(HierarchyRecordSize);
        }
        return false;
    }

    const int recordCount = chunk.size() / HierarchyRecordSize;
    dataset->root = std::make_unique<OctreeNode>();
    dataset->root->id = "r";
    dataset->root->bounds = metadata.bounds;
    dataset->root->level = 0;
    dataset->root->type = OctreeNodeType::Proxy;
    dataset->root->byteOffset = 0;
    dataset->root->byteSize = metadata.hierarchy.firstChunkSize;

    std::vector<OctreeNode*> nodes;
    nodes.reserve(static_cast<std::size_t>(recordCount));
    nodes.push_back(dataset->root.get());

    const uchar* bytes = reinterpret_cast<const uchar*>(chunk.constData());
    for (int recordIndex = 0; recordIndex < recordCount; ++recordIndex) {
        if (recordIndex >= static_cast<int>(nodes.size())) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Hierarchy record %1 in %2 has no matching BFS node.")
                                    .arg(recordIndex)
                                    .arg(hierarchyPath);
            }
            return false;
        }

        OctreeNode* node = nodes[recordIndex];
        const uchar* record = bytes + (recordIndex * HierarchyRecordSize);
        const std::uint8_t recordType = record[0];
        const std::uint8_t childMask = record[1];
        const std::uint32_t numPoints = qFromLittleEndian<std::uint32_t>(record + 2);
        const std::uint64_t byteOffset = qFromLittleEndian<std::uint64_t>(record + 6);
        const std::uint64_t byteSize = qFromLittleEndian<std::uint64_t>(record + 14);

        if (!octreeNodeTypeFromRecord(recordType, &node->type)) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Invalid hierarchy node type %1 at record %2 in %3.")
                                    .arg(recordType)
                                    .arg(recordIndex)
                                    .arg(hierarchyPath);
            }
            return false;
        }

        node->childMask = childMask;
        node->pointCount = byteSize == 0 ? 0 : numPoints;
        node->byteOffset = byteOffset;
        node->byteSize = byteSize;
        node->loadState = OctreeNodeLoadState::Unloaded;

        dataset->firstChunkPointCount += node->pointCount;
        if (node->type == OctreeNodeType::Proxy) {
            ++dataset->proxyNodeCount;
            continue;
        }

        for (int childIndex = 0; childIndex < 8; ++childIndex) {
            const std::uint8_t childBit = static_cast<std::uint8_t>(1u << childIndex);
            if ((childMask & childBit) == 0) {
                continue;
            }

            auto child = std::make_unique<OctreeNode>();
            child->id = node->id + static_cast<char>('0' + childIndex);
            child->level = node->level + 1;
            child->bounds = childBounds(node->bounds, childIndex);
            child->loadState = OctreeNodeLoadState::Unloaded;

            nodes.push_back(child.get());
            node->children[childIndex] = std::move(child);
        }
    }

    if (nodes.size() != static_cast<std::size_t>(recordCount)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("First hierarchy chunk in %1 ended before all declared child records were loaded.")
                                .arg(hierarchyPath);
        }
        return false;
    }

    dataset->hierarchyRecordsLoaded = static_cast<std::uint64_t>(recordCount);
    dataset->maxLoadedLevel = maxLoadedLevel(*dataset->root);
    return true;
}
