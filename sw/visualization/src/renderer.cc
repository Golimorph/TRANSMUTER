#include "renderer.h"

#include <GL/gl.h>
#include <GL/glu.h>

#include <cmath>
#include <cstddef>

#include "trace.h"

namespace transmuter::visualization
{

// We use a static or global helper because GLFW is a C-API
// and doesn't know about C++ 'this' pointers easily.
static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    // Retrieve the Renderer instance attached to this window
    Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer)
    {
        renderer->handleScroll(yoffset);
    }
}

Renderer::Renderer(int width, int height, const std::string& title)
    : m_width(width), m_height(height), m_rotX(20.0f), m_rotY(45.0f)
{
    if (!glfwInit())
    {
        ERROR("Renderer: GLFW Init Failed");
        return;
    }

    m_window = glfwCreateWindow(m_width, m_height, title.c_str(), NULL, NULL);
    if (!m_window)
    {
        ERROR("Renderer: Window Creation Failed");
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(m_window);

    // Industrial "Studio Gray" background
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    // Scroll callback for zooming
    glfwSetWindowUserPointer(m_window, this);  // Attach 'this' so the callback can find it
    glfwSetScrollCallback(m_window, scroll_callback);

    INFO("Renderer: OpenGL Legacy Pipeline initialized (" + std::to_string(width) + "x" +
         std::to_string(height) + ")");
}

Renderer::~Renderer()
{
    if (m_window) glfwDestroyWindow(m_window);
    glfwTerminate();
}

void Renderer::handleInput()
{
    double currentX, currentY;
    glfwGetCursorPos(m_window, &currentX, &currentY);

    if (glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        // 1. Calculate how far the mouse moved since the last frame
        float deltaX = static_cast<float>(currentX - m_lastMouseX);
        float deltaY = static_cast<float>(currentY - m_lastMouseY);

        // 2. Rotate based on that movement
        // Sensitivity (0.5f) controls how fast it spins
        m_rotY += deltaX * 0.5f;
        m_rotX += deltaY * 0.5f;
    }

    // 3. Always update the 'last' position so the next frame knows where we were
    m_lastMouseX = currentX;
    m_lastMouseY = currentY;
}

void Renderer::handleScroll(double yoffset)
{
    // Adjust sensitivity as needed (5.0f or 10.0f usually feels good)
    m_zoom += static_cast<float>(yoffset) * 10.0f;

    // Optional: Clamp the zoom so you don't fly inside the box or too far away
    if (m_zoom > -10.0f) m_zoom = -10.0f;
    if (m_zoom < -1000.0f) m_zoom = -1000.0f;
}

void Renderer::beginFrame()
{
    handleInput();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)m_width / (double)m_height, 1.0, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, m_zoom);
    glRotatef(m_rotX, 1.0f, 0.0f, 0.0f);
    glRotatef(m_rotY, 0.0f, 1.0f, 0.0f);
}

void Renderer::drawWorkpiece(const geometry::PointCloud& workpiece)
{
    workpiece.copyTo(m_pointCloudSnapshot);

    glDisable(GL_LIGHTING);
    glPointSize(3.0f);
    glBegin(GL_POINTS);

    for (const auto& v : m_pointCloudSnapshot)
    {
        // Keep the changing workpiece as the visually dominant shape.
        glColor3f(std::abs(v.normal.x()), std::abs(v.normal.y()), std::abs(v.normal.z()));
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
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
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Renderer::drawTool(const geometry::PointCloud& toolModel, const Eigen::Vector3f& pos,
                        const Eigen::Vector3f& normal)
{
    toolModel.copyTo(m_pointCloudSnapshot);

    glPushMatrix();
    glTranslatef(pos.x(), pos.y(), pos.z());

    Eigen::Vector3f zAxis(0.0f, 0.0f, 1.0f);
    auto q = Eigen::Quaternionf::FromTwoVectors(zAxis, normal.normalized());
    Eigen::AngleAxisf aa(q);
    glRotatef(aa.angle() * 180.0f / M_PI, aa.axis().x(), aa.axis().y(), aa.axis().z());

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.1f, 0.1f);  // Solid Red for the tool as requested

    for (const auto& v : m_pointCloudSnapshot)
    {
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glPopMatrix();
}

void Renderer::drawStock(const geometry::PointCloud& stock)
{
    stock.copyTo(m_pointCloudSnapshot);

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
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Renderer::drawModel(const geometry::PointCloud& model)
{
    model.copyTo(m_pointCloudSnapshot);

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
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
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
