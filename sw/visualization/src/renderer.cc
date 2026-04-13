#include "renderer.h"

#include <GL/gl.h>
#include <GL/glu.h>

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

    // We take a snapshot of the tool model's vertices to minimize time spent in the critical
    // rendering loop
    static std::vector<transmuter::geometry::Vertex> workPieceRenderSnapshot;
    workpiece->copyTo(workPieceRenderSnapshot);

    for (const auto& v : workPieceRenderSnapshot)
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
    for (const auto& v : workPieceRenderSnapshot)
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

    // We take a snapshot of the tool model's vertices to minimize time spent in the critical
    // rendering loop
    static std::vector<transmuter::geometry::Vertex> toolPathRenderSnapshot;
    toolPath->copyTo(toolPathRenderSnapshot);
    for (const auto& v : toolPathRenderSnapshot)
    {
        // Path normals also get colors so we can see tool orientation history
        glColor3f(v.normal.x() * 0.5f + 0.5f, v.normal.y() * 0.5f + 0.5f,
                  v.normal.z() * 0.5f + 0.5f);
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();

    glBegin(GL_LINES);
    for (const auto& v : toolPathRenderSnapshot)
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

    // We take a snapshot of the tool model's vertices to minimize time spent in the critical
    // rendering loop
    static std::vector<transmuter::geometry::Vertex> toolModelRenderSnapshot;
    toolModel->copyTo(toolModelRenderSnapshot);
    for (const auto& v : toolModelRenderSnapshot)
    {
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glPopMatrix();
}

void Renderer::drawStock(std::shared_ptr<geometry::PointCloud> stock)
{
    if (!stock)
    {
        ERROR("stock is null ptr");
    }

    glDisable(GL_LIGHTING);
    glPointSize(2.0f);  // Smaller points for boundaries
    glBegin(GL_POINTS);

    // Faint gray color for stock boundary
    glColor3f(0.2f, 0.2f, 0.2f);

    static std::vector<transmuter::geometry::Vertex> stockRenderSnapshot;
    stock->copyTo(stockRenderSnapshot);
    for (const auto& v : stockRenderSnapshot)
    {
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
}

void Renderer::drawModel(std::shared_ptr<geometry::PointCloud> model)
{
    if (!model)
    {
        ERROR("model is null ptr");
    }

    glDisable(GL_LIGHTING);
    glPointSize(2.0f);  // Smaller points for boundaries
    glBegin(GL_POINTS);

    // Faint blue color for model boundary
    glColor3f(0.3f, 0.3f, 1.0f);

    static std::vector<transmuter::geometry::Vertex> modelRenderSnapshot;
    model->copyTo(modelRenderSnapshot);
    for (const auto& v : modelRenderSnapshot)
    {
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
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