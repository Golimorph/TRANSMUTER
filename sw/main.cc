#include <GL/glew.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "box.h"
#include "pointCloud.h"

using namespace transmuter::geometry;

/**
 * @brief Renders the generated point cloud and its surface normals.
 * Uses color-coding to verify that the normals are pointing in the
 * correct 5-axis directions.
 */
void drawPointCloud(const PointCloud& cloud)
{
    // 1. Draw the actual points
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    for (const auto& v : cloud.getVertices())
    {
        // Map Normal absolute values to RGB for visual debugging
        // This helps identify if X, Y, or Z faces are oriented correctly
        glColor3f(std::abs(v.normal.x()), std::abs(v.normal.y()), std::abs(v.normal.z()));

        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();

    // 2. Draw "Normal Stings" (Surface Orientation)
    // Essential for verifying B-axis flip logic later.
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glColor3f(0.8f, 0.8f, 0.8f);  // Light gray for the needles
    for (const auto& v : cloud.getVertices())
    {
        // Start at vertex position
        glVertex3f(v.pos.x(), v.pos.y(), v.pos.z());

        // End 5mm away in the direction of the normal
        glVertex3f(v.pos.x() + v.normal.x() * 5.0f, v.pos.y() + v.normal.y() * 5.0f,
                   v.pos.z() + v.normal.z() * 5.0f);
    }
    glEnd();
}

int main()
{
    if (!glfwInit()) return -1;

    // Create window
    GLFWwindow* window =
        glfwCreateWindow(1024, 768, "The Transmuter: Point Cloud Alpha", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glewInit();

    // Set background and depth testing
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    // --- GEOMETRY GENERATION ---
    // Generate a 100mm cube with 5mm spacing
    Eigen::Vector3f cubeDims(100.0f, 100.0f, 100.0f);
    PointCloud workPiece = shapes::Box::generate(cubeDims, 5.0f);

    std::cout << "Bernardo Engine: Initialized with " << workPiece.size() << " points."
              << std::endl;

    float rotation = 0.0f;

    while (!glfwWindowShouldClose(window))
    {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, (double)width / (double)height, 1.0, 1000.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Move camera back to see the whole 100mm part
        gluLookAt(200.0, 150.0, 200.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        glRotatef(rotation, 0, 1, 0);

        drawPointCloud(workPiece);

        rotation += 0.4f;

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}