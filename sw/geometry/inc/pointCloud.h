#ifndef TRANSMUTER_GEOMETRY_POINT_CLOUD_H
#define TRANSMUTER_GEOMETRY_POINT_CLOUD_H

#include <algorithm>
#include <mutex>
#include <vector>

#include "trace.h"
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
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vertices.reserve(capacity);
    }

    void addVertex(const Vertex& v)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vertices.push_back(v);
    }

    Vertex getVertex(size_t index) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (index >= m_vertices.size())
        {
            ERROR("Vertex index out of bounds.");
        }
        return m_vertices[index];
    }

    // Thread-safe copy of vertex data for rendering
    void copyTo(std::vector<Vertex>& outBuffer) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        outBuffer = m_vertices;  // Fast vector assignment/copy
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_vertices.size();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_vertices.clear();
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
    std::vector<Vertex> m_vertices;
    mutable std::mutex m_mutex;
};

}  // namespace transmuter::geometry

#endif