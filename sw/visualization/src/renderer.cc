#include "renderer.h"

#include <GL/gl.h>
#include <GL/glu.h>

#include "trace.h"

namespace transmuter::visualization
{

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
    if (glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        m_rotY += 0.5f;
    }
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
    glTranslatef(0.0f, 0.0f, -300.0f);
    glRotatef(m_rotX, 1.0f, 0.0f, 0.0f);
    glRotatef(m_rotY, 0.0f, 1.0f, 0.0f);
}

void Renderer::drawWorkpiece(std::shared_ptr<geometry::PointCloud> workpiece)
{
    if (!workpiece)
    {
        ERROR("workpiece is null ptr");
    }

    // Ensure lighting doesn't override our normal-to-RGB mapping
    glDisable(GL_LIGHTING);

    // 1. Draw Points with Normal-to-RGB color mapping
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for (const auto& v : workpiece->getVertices())
    {
        // Absolute value of normals maps well to RGB for visual debugging
        // |X| -> Red, |Y| -> Green, |Z| -> Blue
        glColor3f(std::abs(v.normal.x()), std::abs(v.normal.y()), std::abs(v.normal.z()));
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();

    // 2. Draw Surface Normal "Needles"
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (const auto& v : workpiece->getVertices())
    {
        glColor3f(std::abs(v.normal.x()), std::abs(v.normal.y()), std::abs(v.normal.z()));

        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());

        // Project a 5mm line segment along the normal vector
        glVertex3f(v.pos.x() + v.normal.x() * 5.0f, v.pos.y() + v.normal.y() * 5.0f,
                   v.pos.z() + v.normal.z() * 5.0f);
    }
    glEnd();
}

void Renderer::drawToolPath(std::shared_ptr<geometry::PointCloud> toolPath)
{
    if (!toolPath)
    {
        ERROR("toolPath is null ptr");
    }

    glDisable(GL_LIGHTING);
    glBegin(GL_LINE_STRIP);
    for (const auto& v : toolPath->getVertices())
    {
        // Path normals also get colors so we can see tool orientation history
        glColor3f(v.normal.x() * 0.5f + 0.5f, v.normal.y() * 0.5f + 0.5f,
                  v.normal.z() * 0.5f + 0.5f);
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();

    glBegin(GL_LINES);
    for (const auto& v : toolPath->getVertices())
    {
        glColor3f(std::abs(v.normal.x()), std::abs(v.normal.y()), std::abs(v.normal.z()));

        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());

        // Project a 5mm line segment along the normal vector
        glVertex3f(v.pos.x() + v.normal.x() * 5.0f, v.pos.y() + v.normal.y() * 5.0f,
                   v.pos.z() + v.normal.z() * 5.0f);
    }
    glEnd();
}

void Renderer::drawTool(std::shared_ptr<geometry::PointCloud> toolModel, const Eigen::Vector3f& pos,
                        const Eigen::Vector3f& normal)
{
    if (!toolModel)
    {
        ERROR("toolModel is null ptr");
    }

    glPushMatrix();
    glTranslatef(pos.x(), pos.y(), pos.z());

    Eigen::Vector3f zAxis(0.0f, 0.0f, 1.0f);
    auto q = Eigen::Quaternionf::FromTwoVectors(zAxis, normal.normalized());
    Eigen::AngleAxisf aa(q);
    glRotatef(aa.angle() * 180.0f / M_PI, aa.axis().x(), aa.axis().y(), aa.axis().z());

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.1f, 0.1f);  // Solid Red for the tool as requested
    for (const auto& v : toolModel->getVertices())
    {
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glPopMatrix();
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