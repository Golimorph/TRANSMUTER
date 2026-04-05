#include "visualizer.h"

#include <GL/glu.h>

#include <cmath>
#include <iostream>

namespace transmuter::visualization
{

Visualizer::Visualizer(int width, int height, const std::string& title)
    : m_window(nullptr), m_width(width), m_height(height)
{
    if (!glfwInit())
    {
        std::cerr << "CRITICAL: GLFW initialization failed." << std::endl;
        return;
    }

    m_window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
    if (!m_window)
    {
        std::cerr << "CRITICAL: Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(m_window);

    // glewExperimental is often needed for modern OpenGL features
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "CRITICAL: GLEW initialization failed." << std::endl;
        return;
    }

    // Industrial "Studio Gray" background
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glEnable(GL_DEPTH_TEST);
}

Visualizer::~Visualizer()
{
    if (m_window)
    {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

bool Visualizer::shouldClose() const
{
    return m_window ? glfwWindowShouldClose(m_window) : true;
}

void Visualizer::beginFrame()
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

void Visualizer::drawPointCloud(const geometry::PointCloud& cloud, float rotationAngle)
{
    glPushMatrix();
    glRotatef(rotationAngle, 0, 1, 0);

    // Draw Points with Normal-to-RGB color mapping
    // +X (Right) = Red, +Y (Up) = Green, +Z (Forward) = Blue
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for (const auto& v : cloud.getVertices())
    {
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

void Visualizer::endFrame()
{
    if (!m_window) return;
    glfwSwapBuffers(m_window);
    glfwPollEvents();
}

}  // namespace transmuter::visualization