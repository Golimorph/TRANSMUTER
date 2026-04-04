#ifndef TRANSMUTER_GEOMETRY_POINT_CLOUD_H
#define TRANSMUTER_GEOMETRY_POINT_CLOUD_H

#include <algorithm>
#include <vector>

#include "vertex.h"

namespace transmuter::geometry
{

/**
 * @brief A container for 3D surface data.
 *
 * Optimized for high-density 5-axis toolpath generation.
 * Use reserve() to avoid reallocations during geometry parsing.
 */
class PointCloud
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
        m_vertices.reserve(capacity);
    }

    void addVertex(const Vertex& v)
    {
        m_vertices.push_back(v);
    }

    const std::vector<Vertex>& getVertices() const
    {
        return m_vertices;
    }

    size_t size() const
    {
        return m_vertices.size();
    }

    void clear()
    {
        m_vertices.clear();
    }

    void finalize()
    {
        m_vertices.shrink_to_fit();
    }

   private:
    std::vector<Vertex> m_vertices;
};

}  // namespace transmuter::geometry

#endif