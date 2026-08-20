#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

enum class PointAttributeType {
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Int64,
    UInt64,
    Float,
    Double,
    Unknown
};

struct PointAttribute {
    std::string name;
    std::string description;
    std::uint32_t sizeBytes = 0;
    std::uint32_t numElements = 0;
    std::uint32_t elementSizeBytes = 0;
    PointAttributeType type = PointAttributeType::Unknown;
    std::vector<double> min;
    std::vector<double> max;
    std::vector<double> scale;
    std::vector<double> offset;
};

class PointAttributes {
public:
    void add(PointAttribute attribute)
    {
        m_attributes.push_back(std::move(attribute));
    }

    const PointAttribute* find(std::string_view name) const
    {
        for (const PointAttribute& attribute : m_attributes) {
            if (attribute.name == name) {
                return &attribute;
            }
        }

        return nullptr;
    }

    std::uint32_t pointRecordSizeBytes() const
    {
        std::uint32_t size = 0;
        for (const PointAttribute& attribute : m_attributes) {
            size += attribute.sizeBytes;
        }

        return size;
    }

    const std::vector<PointAttribute>& items() const
    {
        return m_attributes;
    }

private:
    std::vector<PointAttribute> m_attributes;
};
