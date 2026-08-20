#pragma once

#include <osg/Group>
#include <osg/Node>
#include <osg/ref_ptr>

#include <QString>

#include <cstdint>
#include <string>

class SceneManager
{
public:
    SceneManager();

    osg::Group* root() const;

    bool loadPointCloud(const std::string& nativeFilePath,
                        const QString& displayFilePath,
                        float pointSize,
                        QString* errorMessage);
    void clear();
    void setPointSize(float pointSize);

    QString currentFilePath() const;
    std::uint64_t pointCount() const;

private:
    void applyPointCloudState(osg::Node* node, float pointSize);
    std::uint64_t countPointsAndPrepareGeometry(osg::Node* node) const;

    osg::ref_ptr<osg::Group> m_root;
    osg::ref_ptr<osg::Node> m_pointCloudNode;
    QString m_currentFilePath;
    std::uint64_t m_pointCount = 0;
};
