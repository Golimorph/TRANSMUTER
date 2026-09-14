#include "renderer.h"

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
    if (renderer)
    {
        renderer->handleScroll(yOffset);
    }
}

Renderer::Renderer(int width, int height, const std::string& title)
    : m_width(width), m_height(height), m_rotX(20.0), m_rotY(45.0)
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

    glfwMakeContextCurrent(m_window);
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glfwSetWindowUserPointer(m_window, this);
    glfwSetScrollCallback(m_window, scrollCallback);

    INFO("Renderer: OpenGL Legacy Pipeline initialized (" + std::to_string(width) + "x" + std::to_string(height) +
         ")");
}

Renderer::~Renderer()
{
    if (m_window)
    {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

void Renderer::handleInput()
{
    double currentX = 0.0;
    double currentY = 0.0;
    glfwGetCursorPos(m_window, &currentX, &currentY);

    if (glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        const double deltaX = currentX - m_lastMouseX;
        const double deltaY = currentY - m_lastMouseY;
        m_rotY += deltaX * 0.5;
        m_rotX += deltaY * 0.5;
    }

    m_lastMouseX = currentX;
    m_lastMouseY = currentY;
}

void Renderer::handleScroll(double yOffset)
{
    m_zoom = std::clamp(m_zoom + yOffset * 10.0, -1000.0, -10.0);
}

void Renderer::beginFrame()
{
    handleInput();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, static_cast<double>(m_width) / m_height, 1.0, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, m_zoom);
    glRotated(m_rotX, 1.0, 0.0, 0.0);
    glRotated(m_rotY, 0.0, 1.0, 0.0);
}

void Renderer::drawWorkpiece(const geometry::GeometryIf& workpiece)
{
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
}

void Renderer::drawToolPath(const geometry::PointCloud& toolPath)
{
    toolPath.copyTo(m_pointCloudSnapshot);

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glLineWidth(1.0f);
    glBegin(GL_LINE_STRIP);

    for (const auto& v : m_pointCloudSnapshot)
    {
        glColor4f(0.35f, 0.75f, 0.85f, 0.10f);
        glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
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

void Renderer::drawStock(const geometry::GeometryIf& stock)
{
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
    glfwPollEvents();
}

bool Renderer::shouldClose() const
{
    return glfwWindowShouldClose(m_window);
}

}  // namespace transmuter::visualization
