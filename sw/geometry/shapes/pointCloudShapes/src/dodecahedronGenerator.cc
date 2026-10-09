#include "dodecahedronGenerator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

#include "trace.h"

namespace transmuter::geometry::pointcloudshapes
{

namespace
{

using DodecahedronVertices = std::array<Eigen::Vector3d, 20>;

struct Face
{
    std::vector<Eigen::Vector3d> vertices;
    Eigen::Vector3d normal;
    double offset;
};

void validateDodecahedron(double circumradius, double density)
{
    if (circumradius <= 0.0 || density <= 0.0)
    {
        ERROR("Dodecahedron circumradius and density must be greater than zero.");
    }
}

DodecahedronVertices canonicalVertices()
{
    const double phi = std::numbers::phi;
    const double inversePhi = 1.0 / phi;
    return {{
        {-1.0, -1.0, -1.0},
        {-1.0, -1.0, 1.0},
        {-1.0, 1.0, -1.0},
        {-1.0, 1.0, 1.0},
        {1.0, -1.0, -1.0},
        {1.0, -1.0, 1.0},
        {1.0, 1.0, -1.0},
        {1.0, 1.0, 1.0},
        {0.0, -inversePhi, -phi},
        {0.0, -inversePhi, phi},
        {0.0, inversePhi, -phi},
        {0.0, inversePhi, phi},
        {-inversePhi, -phi, 0.0},
        {-inversePhi, phi, 0.0},
        {inversePhi, -phi, 0.0},
        {inversePhi, phi, 0.0},
        {-phi, 0.0, -inversePhi},
        {-phi, 0.0, inversePhi},
        {phi, 0.0, -inversePhi},
        {phi, 0.0, inversePhi},
    }};
}

DodecahedronVertices scaledVertices(double circumradius)
{
    DodecahedronVertices vertices = canonicalVertices();
    const double scale = circumradius / std::sqrt(3.0);
    for (auto& vertex : vertices)
    {
        vertex *= scale;
    }
    return vertices;
}

std::array<Eigen::Vector3d, 12> faceNormals()
{
    const double phi = std::numbers::phi;
    std::array<Eigen::Vector3d, 12> normals{{
        {0.0, phi, 1.0},
        {0.0, phi, -1.0},
        {0.0, -phi, 1.0},
        {0.0, -phi, -1.0},
        {1.0, 0.0, phi},
        {1.0, 0.0, -phi},
        {-1.0, 0.0, phi},
        {-1.0, 0.0, -phi},
        {phi, 1.0, 0.0},
        {phi, -1.0, 0.0},
        {-phi, 1.0, 0.0},
        {-phi, -1.0, 0.0},
    }};
    for (auto& normal : normals)
    {
        normal.normalize();
    }
    return normals;
}

Eigen::Vector3d faceCenter(const std::vector<Eigen::Vector3d>& vertices)
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& vertex : vertices)
    {
        center += vertex;
    }
    return center / static_cast<double>(vertices.size());
}

Face buildFace(const DodecahedronVertices& vertices, const Eigen::Vector3d& normal, double tolerance)
{
    const auto furthest = std::max_element(vertices.begin(), vertices.end(),
        [&](const auto& left, const auto& right) { return normal.dot(left) < normal.dot(right); });
    Face face{.vertices = {}, .normal = normal, .offset = normal.dot(*furthest)};
    std::copy_if(vertices.begin(), vertices.end(), std::back_inserter(face.vertices), [&](const auto& vertex)
        { return std::abs(face.normal.dot(vertex) - face.offset) <= tolerance; });
    if (face.vertices.size() != 5)
    {
        ERROR("Could not construct a regular dodecahedron face.");
    }
    return face;
}

void orderFaceVertices(Face& face)
{
    const Eigen::Vector3d center = faceCenter(face.vertices);
    const Eigen::Vector3d xAxis = (face.vertices.front() - center).normalized();
    const Eigen::Vector3d yAxis = face.normal.cross(xAxis);
    std::sort(face.vertices.begin(), face.vertices.end(),
        [&](const auto& left, const auto& right)
        {
            const Eigen::Vector3d leftOffset = left - center;
            const Eigen::Vector3d rightOffset = right - center;
            return std::atan2(leftOffset.dot(yAxis), leftOffset.dot(xAxis)) <
                   std::atan2(rightOffset.dot(yAxis), rightOffset.dot(xAxis));
        });
}

std::vector<Face> dodecahedronFaces(double circumradius)
{
    const DodecahedronVertices vertices = scaledVertices(circumradius);
    std::vector<Face> faces;
    faces.reserve(12);
    for (const auto& normal : faceNormals())
    {
        Face face = buildFace(vertices, normal, circumradius * 1e-9);
        orderFaceVertices(face);
        faces.push_back(std::move(face));
    }
    return faces;
}

void addTriangleSamples(PointCloud& cloud, const Eigen::Vector3d& first, const Eigen::Vector3d& second, const Eigen::Vector3d& third, const Eigen::Vector3d& normal, double density)
{
    const double longestEdge = std::max({(second - first).norm(), (third - second).norm(),
        (first - third).norm()});
    const int subdivisions = static_cast<int>(std::max(1.0, std::ceil(longestEdge / density)));
    for (int firstStep = 0; firstStep <= subdivisions; ++firstStep)
    {
        for (int secondStep = 0; secondStep <= subdivisions - firstStep; ++secondStep)
        {
            const double firstWeight = static_cast<double>(firstStep) / subdivisions;
            const double secondWeight = static_cast<double>(secondStep) / subdivisions;
            cloud.append(Vertex(first + firstWeight * (second - first) + secondWeight * (third - first), normal));
        }
    }
}

void addFaceSamples(PointCloud& cloud, const Face& face, double density)
{
    const Eigen::Vector3d center = faceCenter(face.vertices);
    for (size_t index = 0; index < face.vertices.size(); ++index)
    {
        const Eigen::Vector3d& current = face.vertices[index];
        const Eigen::Vector3d& next = face.vertices[(index + 1) % face.vertices.size()];
        addTriangleSamples(cloud, center, current, next, face.normal, density);
    }
}

bool isInside(const Eigen::Vector3d& point, const std::vector<Face>& faces)
{
    return std::all_of(faces.begin(), faces.end(),
        [&](const Face& face) { return face.normal.dot(point) <= face.offset; });
}

Eigen::Vector3d closestFaceNormal(const Eigen::Vector3d& point, const std::vector<Face>& faces)
{
    const auto closest = std::min_element(faces.begin(), faces.end(), [&](const Face& left, const Face& right)
        { return left.offset - left.normal.dot(point) < right.offset - right.normal.dot(point); });
    return closest->normal;
}

void addSolidSamples(PointCloud& cloud, double circumradius, double density, const std::vector<Face>& faces)
{
    const int steps = static_cast<int>(std::max(2.0, std::ceil(2.0 * circumradius / density)));
    for (int xIndex = 0; xIndex <= steps; ++xIndex)
    {
        const double x = -circumradius + 2.0 * circumradius * xIndex / steps;
        for (int yIndex = 0; yIndex <= steps; ++yIndex)
        {
            const double y = -circumradius + 2.0 * circumradius * yIndex / steps;
            for (int zIndex = 0; zIndex <= steps; ++zIndex)
            {
                const double z = -circumradius + 2.0 * circumradius * zIndex / steps;
                const Eigen::Vector3d point(x, y, z);
                if (isInside(point, faces))
                {
                    cloud.append(Vertex(point, closestFaceNormal(point, faces)));
                }
            }
        }
    }
}

}  // namespace

std::shared_ptr<PointCloud> DodecahedronGenerator::generate(double circumradius, double density)
{
    validateDodecahedron(circumradius, density);
    auto cloud = std::make_shared<PointCloud>();
    for (const auto& face : dodecahedronFaces(circumradius))
    {
        addFaceSamples(*cloud, face, density);
    }
    cloud->finalize();
    return cloud;
}

std::shared_ptr<PointCloud> DodecahedronGenerator::generateSolid(double circumradius, double density)
{
    validateDodecahedron(circumradius, density);
    const std::vector<Face> faces = dodecahedronFaces(circumradius);
    auto cloud = std::make_shared<PointCloud>();
    addSolidSamples(*cloud, circumradius, density, faces);
    cloud->finalize();
    return cloud;
}

}  // namespace transmuter::geometry::pointcloudshapes
