#ifndef TRANSMUTER_GEOMETRY_POINT_CLOUD_H
#define TRANSMUTER_GEOMETRY_POINT_CLOUD_H

#include <algorithm>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "../../if/geometryIf.h"

namespace transmuter::geometry
{

/**
 * @brief A point-sampled implementation of GeometryIf.
 *
 * Samples may describe a surface or a filled volume. Geometry consumers should depend on
 * GeometryIf; direct point access is reserved for point-cloud backends.
 */
class PointCloud : public GeometryIf
{
   public:
    PointCloud() = default;

    /**
     * @brief Pre-allocate memory for the entire workpiece.
     * Prevents the performance hit of dynamic resizing.
     */
    explicit PointCloud(size_t capacity)
    {
        m_vertices.reserve(capacity);
    }

    void reserve(size_t capacity)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vertices.reserve(capacity);
    }

    void append(const Vertex& vertex)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vertices.push_back(vertex);
        m_minimumPointSpacing.reset();
    }

    void copyTo(std::vector<Vertex>& output) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        output = m_vertices;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vertices.clear();
        m_minimumPointSpacing.reset();
    }

    /** @brief Remove every vertex accepted by predicate in one locked operation. */
    template <typename Predicate>
    void eraseIf(Predicate&& predicate)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vertices.erase(std::remove_if(m_vertices.begin(), m_vertices.end(), std::forward<Predicate>(predicate)),
            m_vertices.end());
        m_minimumPointSpacing.reset();
    }

    /** @brief Make an independent geometry snapshot. */
    std::shared_ptr<GeometryIf> clone() const override
    {
        auto result = std::make_shared<PointCloud>();
        std::lock_guard<std::mutex> lock(m_mutex);
        result->m_vertices = m_vertices;
        result->m_minimumPointSpacing = m_minimumPointSpacing;
        return result;
    }

    bool isInside(const Eigen::Vector3d& point) const override;
    bool intersects(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform = Eigen::Affine3d::Identity()) const override;
    void combine(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform = Eigen::Affine3d::Identity()) override;
    void cut(const GeometryIf& cuttingGeometry, const Eigen::Affine3d& cuttingToThis = Eigen::Affine3d::Identity()) override;
    void cut(const GeometryIf& cuttingGeometry, const std::vector<Eigen::Affine3d>& cuttingToThis) override;
    GeometryBounds bounds() const override;
    double maximumDistanceFromAxis(const Eigen::Vector3d& axisPoint, const Eigen::Vector3d& axisDirection) const override;

    void copyVisualizationVertices(std::vector<Vertex>& output) const override
    {
        copyTo(output);
    }

    void finalize()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vertices.shrink_to_fit();
    }

    bool isEmpty() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_vertices.empty();
    }

   private:
    double minimumPointSpacingLocked() const;

    std::vector<Vertex> m_vertices;
    mutable std::mutex m_mutex;
    mutable std::optional<double> m_minimumPointSpacing;
};

}  // namespace transmuter::geometry

#endif
