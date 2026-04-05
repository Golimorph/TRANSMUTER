#include "renderer.h"

#include <GL/glu.h>
#include <GLFW/glfw3.h>

#include <cmath>
#include <string>

#include "trace.h"

namespace transmuter::visualization
{

Renderer::Renderer(int width, int height, const std::string& title)
    : m_window(nullptr), m_width(width), m_height(height)
{
    if (!glfwInit())
    {
        SHUTDOWN_ERROR("GLFW initialization failed.");
    }

    // Set hints for a stable legacy context if needed
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    m_window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
    if (!m_window)
    {
        glfwTerminate();
        SHUTDOWN_ERROR("Failed to create GLFW window.");
    }

    glfwMakeContextCurrent(m_window);

    // Industrial "Studio Gray" background
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    INFO("Renderer: OpenGL Legacy Pipeline initialized (" + std::to_string(width) + "x" +
         std::to_string(height) + ")");
}

Renderer::~Renderer()
{
    if (m_window)
    {
        glfwDestroyWindow(m_window);
        INFO("Renderer: Window destroyed.");
    }
    glfwTerminate();
}

bool Renderer::shouldClose() const
{
    return m_window ? glfwWindowShouldClose(m_window) : true;
}

void Renderer::beginFrame()
{
    if (!m_window) return;

    glfwGetFramebufferSize(m_window, &m_width, &m_height);
    glViewport(0, 0, m_width, m_height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 1. Setup Projection (The Lens)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = (m_height > 0) ? (double)m_width / (double)m_height : 1.0;
    gluPerspective(45.0, aspect, 1.0, 1000.0);

    // 2. Setup ModelView (The Camera)
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Position camera at a comfortable viewing distance for a 100mm part
    // Using industrial standard Y-up orientation
    gluLookAt(200.0, 150.0, 200.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

void Renderer::drawPointCloud(const geometry::PointCloud& cloud, float rotationAngle)
{
    glPushMatrix();
    glRotatef(rotationAngle, 0, 1, 0);

    // Draw Points with Normal-to-RGB color mapping
    // +X (Right) = Red, +Y (Up) = Green, +Z (Forward) = Blue
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for (const auto& v : cloud.getVertices())
    {
        // Absolute value of normals maps well to RGB for visual debugging
        glColor3f(std::abs(v.normal.x()), std::abs(v.normal.y()), std::abs(v.normal.z()));
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();

    // Draw Surface Normal "Needles"
    // Essential for visually confirming the part's orientation relative to the machine B-axis
    glLineWidth(1.0f);
    glColor3f(0.8f, 0.8f, 0.8f);
    glBegin(GL_LINES);
    for (const auto& v : cloud.getVertices())
    {
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());

        // Project a 5mm line segment along the normal vector
        glVertex3f(v.pos.x() + v.normal.x() * 5.0f, v.pos.y() + v.normal.y() * 5.0f,
                   v.pos.z() + v.normal.z() * 5.0f);
    }
    glEnd();

    glPopMatrix();
}

void Renderer::drawToolPath(const geometry::PointCloud& path, float rotation)
{
    glPushMatrix();
    glRotatef(rotation, 0.0f, 1.0f, 0.0f);

    const auto& nodes = path.getVertices();

    // 1. Draw the connection lines (The "Trace")
    glBegin(GL_LINE_STRIP);
    glColor3f(0.0f, 1.0f, 0.0f);  // Bright green for the path
    for (const auto& v : nodes)
    {
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();

    // 2. Draw the orientation needles (The 5-Axis "IJK" vectors)
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.5f, 0.0f);  // Orange for tool orientation
    for (const auto& v : nodes)
    {
        // Base of the tool
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
        // Tip of the "needle" showing where the tool points
        glVertex3f(v.pos.x() + v.normal.x() * 15.0f, v.pos.y() + v.normal.y() * 15.0f,
                   v.pos.z() + v.normal.z() * 15.0f);
    }
    glEnd();

    glPopMatrix();
}

void Renderer::endFrame()
{
    if (!m_window) return;
    glfwSwapBuffers(m_window);
    glfwPollEvents();
}

}  // namespace transmuter::visualization