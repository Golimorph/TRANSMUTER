#include <GL/glew.h>
#include <GL/glu.h>  // Ensure GLU is included
#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <print>

using namespace Eigen;

void drawScene(Vector3d force)
{
    // Draw Coordinate Cross
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 0, 0);  // X
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 1, 0);  // Y
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);  // Z
    glEnd();

    // Draw The Transmuter's Force Vector
    glLineWidth(4.0f);
    glBegin(GL_LINES);
    glColor3f(1, 1, 0);  // Yellow
    glVertex3f(0, 0, 0);
    glVertex3f(force.x(), force.y(), force.z());
    glEnd();
}

int main()
{
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(800, 600, "The Transmuter: Visualizer", NULL, NULL);
    if (!window) return -1;
    glfwMakeContextCurrent(window);
    glewInit();

    // Set a "Studio Gray" background so we can see the lines
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    Vector3d force(0.5, 0.7, 0.3);  // A sample vector to "see"

    while (!glfwWindowShouldClose(window))
    {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 1. Setup Projection (The Lens)
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, (double)width / (double)height, 0.1, 100.0);

        // 2. Setup ModelView (The Camera)
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        // Move back 3 units and look at the origin (0,0,0)
        gluLookAt(3.0, 3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

        drawScene(force);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}