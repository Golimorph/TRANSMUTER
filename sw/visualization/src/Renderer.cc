#include "Renderer.h"

#include <GL/gl.h>
#include <GL/glu.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "trace.h"

namespace transmuter::visualization
{

void scrollCallback(GLFWwindow* window, double, double yOffset)
{
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    renderer->handleScroll(yOffset);
}

Renderer::Renderer(int width, int height, const std::string& title)
    : m_title(title), m_width(width), m_height(height)
{
    if (!glfwInit())
    {
        ERROR("Renderer: GLFW Init Failed");
    }

    m_window = glfwCreateWindow(m_width, m_height, title.c_str(), nullptr, nullptr);
    if (!m_window)
    {
        glfwTerminate();
        ERROR("Renderer: Window Creation Failed");
    }

    glfwSetWindowUserPointer(m_window, this);
    glfwSetScrollCallback(m_window, scrollCallback);
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    INFO("Renderer: window created (" + std::to_string(width) + "x" + std::to_string(height) + ")");
}

Renderer::~Renderer()
{
    glfwDestroyWindow(m_window);
    glfwTerminate();
}

bool Renderer::processEvents()
{
    glfwWaitEventsTimeout(0.01);
    if (glfwWindowShouldClose(m_window))
    {
        return false;
    }
    handleInput();
    return true;
}

void Renderer::handleInput()
{
    if (glfwGetKey(m_window, GLFW_KEY_M) == GLFW_PRESS)
    {
        setViewCoordinates(ViewCoordinates::MACHINE);
    }
    if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS)
    {
        setViewCoordinates(ViewCoordinates::WORKPIECE);
    }
    double currentX = 0.0;
    double currentY = 0.0;
    glfwGetCursorPos(m_window, &currentX, &currentY);

    const bool dragging = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    if (dragging && m_dragging)
    {
        const double deltaX = currentX - m_lastMouseX;
        const double deltaY = currentY - m_lastMouseY;
        m_camera.phiDegrees = std::remainder(m_camera.phiDegrees + deltaX * 0.5, 360.0);
        m_camera.thetaDegrees = std::clamp(m_camera.thetaDegrees - deltaY * 0.5, 1.0, 179.0);
    }

    m_dragging = dragging;
    m_lastMouseX = currentX;
    m_lastMouseY = currentY;
}

void Renderer::handleScroll(double yOffset)
{
    m_camera.zoom = std::clamp(m_camera.zoom + yOffset * 10.0, -3000.0, -10.0);
}

void Renderer::beginFrame()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, static_cast<double>(m_width) / m_height, 1.0, 5000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, m_camera.zoom);
    // Compose polar tilt with azimuth about scene Z, without camera roll.
    glRotated(-m_camera.thetaDegrees, 1.0, 0.0, 0.0);
    glRotated(m_camera.phiDegrees, 0.0, 0.0, 1.0);
    if (m_camera.coordinates == ViewCoordinates::MACHINE)
    {
        glTranslated(0.0, 0.0, -120.0);
    }
}

void Renderer::drawWorkpiece(const geometry::GeometryIf& workpiece, const Eigen::Affine3d& transform)
{
    glPushMatrix();
    glMultMatrixd(transform.data());
    workpiece.copyVisualizationVertices(m_pointCloudSnapshot);

    glDisable(GL_LIGHTING);
    glPointSize(3.0f);
    glBegin(GL_POINTS);

    for (const auto& v : m_pointCloudSnapshot)
    {
        // Keep the changing workpiece as the visually dominant shape.
        glColor3d(std::abs(v.normal.x()), std::abs(v.normal.y()), std::abs(v.normal.z()));
        glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glPopMatrix();
}

void Renderer::drawToolPath(const std::vector<geometry::Vertex>& points, const Eigen::Affine3d& transform)
{
    glPushMatrix();
    glMultMatrixd(transform.data());

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glLineWidth(1.5f);
    glBegin(GL_LINE_STRIP);

    for (const auto& point : points)
    {
        glColor4f(0.35f, 0.75f, 0.85f, 0.55f);
        glVertex3d(point.pos.x(), point.pos.y(), point.pos.z());
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glPopMatrix();
}

void Renderer::drawTool(const geometry::GeometryIf& toolModel, const Eigen::Vector3d& pos, const Eigen::Vector3d& normal)
{
    toolModel.copyVisualizationVertices(m_pointCloudSnapshot);

    glPushMatrix();
    glTranslated(pos.x(), pos.y(), pos.z());

    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),
        normal.normalized());
    const Eigen::AngleAxisd angleAxis(rotation);
    const double angleDegrees = angleAxis.angle() * 180.0 / std::numbers::pi;
    glRotated(angleDegrees, angleAxis.axis().x(), angleAxis.axis().y(), angleAxis.axis().z());

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.1f, 0.1f);  // Solid Red for the tool as requested

    for (const auto& v : m_pointCloudSnapshot)
    {
        glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glPopMatrix();
}

void Renderer::drawStock(const geometry::GeometryIf& stock, const Eigen::Affine3d& transform)
{
    glPushMatrix();
    glMultMatrixd(transform.data());
    stock.copyVisualizationVertices(m_pointCloudSnapshot);

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glPointSize(1.0f);
    glBegin(GL_POINTS);

    glColor4f(0.55f, 0.55f, 0.55f, 0.025f);

    // The stock may be volumetric, so draw a sparse reference instead of obscuring the workpiece.
    for (size_t index = 0; index < m_pointCloudSnapshot.size(); index += 8)
    {
        const auto& v = m_pointCloudSnapshot[index];
        glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glPopMatrix();
}

void Renderer::drawModel(const geometry::GeometryIf& model)
{
    model.copyVisualizationVertices(m_pointCloudSnapshot);

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glPointSize(3.0f);
    glBegin(GL_POINTS);

    glColor4f(1.0f, 0.85f, 0.05f, 0.85f);

    for (const auto& v : m_pointCloudSnapshot)
    {
        glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Renderer::endFrame()
{
    glfwSwapBuffers(m_window);
}

ViewCoordinates Renderer::viewCoordinates() const
{
    return m_camera.coordinates;
}
void Renderer::setViewCoordinates(ViewCoordinates coordinates)
{
    if (coordinates != m_camera.coordinates)
    {
        m_camera.zoom = coordinates == ViewCoordinates::MACHINE ? -1300.0 : -300.0;
        m_camera.thetaDegrees = coordinates == ViewCoordinates::MACHINE ? 65.0 : 55.0;
        m_camera.phiDegrees = coordinates == ViewCoordinates::MACHINE ? 0.0 : 45.0;
    }
    m_camera.coordinates = coordinates;
    const std::string title = m_title + (coordinates == ViewCoordinates::MACHINE ? " | Machine view [M]" : " | Workpiece view [W]") +
                              " | W/M: switch frame";
    glfwSetWindowTitle(m_window, title.c_str());
}
void Renderer::drawMachine(const MachineDrawing& drawing)
{
    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    const GLfloat light[] = {0.4f, -0.6f, 1.0f, 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    for (const auto& part : drawing) drawMachinePart(part);
    glPopClientAttrib();
    glPopAttrib();
}

void Renderer::drawMachinePart(const MachineDrawingPart& part)
{
    if (!part.mesh || part.mesh->vertices.empty())
    {
        return;
    }
    glPushMatrix();
    glMultMatrixd(part.transform.data());
    glColor3fv(part.color.data());
    glVertexPointer(3, GL_FLOAT, sizeof(MachineMeshVertex), part.mesh->vertices.front().position.data());
    glNormalPointer(GL_FLOAT, sizeof(MachineMeshVertex), part.mesh->vertices.front().normal.data());
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(part.mesh->triangleIndices.size()), GL_UNSIGNED_INT,
        part.mesh->triangleIndices.data());
    glPopMatrix();
}

}  // namespace transmuter::visualization
